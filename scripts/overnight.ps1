[CmdletBinding()]
param(
    [string]$Root = 'D:\projects\BioSynGenBenchmark',
    [string]$SessionId,
    [string]$LinearSnapshotPath,
    [ValidateRange(0.25, 6.0)]
    [double]$MaxHours = 6.0,
    [switch]$DryRun,
    [switch]$Resume
)

$script:CanonicalRoot = 'D:\projects\BioSynGenBenchmark'
$script:ProjectId = 'P-RES-41'
$script:Queue = @('RES-475', 'RES-476', 'RES-477', 'RES-478', 'RES-479')
$script:AllowedIssueStates = @('Backlog', 'Todo', 'In Progress', 'In Review', 'Done')
$script:MaxAttemptsPerIssue = 3

function Get-FullPath {
    param([Parameter(Mandatory)][string]$Path)
    return [IO.Path]::GetFullPath($Path).TrimEnd([IO.Path]::DirectorySeparatorChar, [IO.Path]::AltDirectorySeparatorChar)
}

function Test-PathInsideRoot {
    param(
        [Parameter(Mandatory)][string]$Path,
        [Parameter(Mandatory)][string]$CanonicalProjectRoot
    )
    $fullPath = Get-FullPath -Path $Path
    $prefix = $CanonicalProjectRoot.TrimEnd('\') + '\'
    return $fullPath.StartsWith($prefix, [StringComparison]::OrdinalIgnoreCase)
}

function Assert-ProjectRoot {
    param([Parameter(Mandatory)][string]$Path)
    $fullRoot = Get-FullPath -Path $Path
    if ($fullRoot -ine $script:CanonicalRoot) {
        throw "Root must resolve to the authorized project path $script:CanonicalRoot."
    }
    if (-not (Test-Path -LiteralPath $fullRoot -PathType Container)) {
        throw "Project root is missing: $fullRoot"
    }
    $rootItem = Get-Item -LiteralPath $fullRoot -Force -ErrorAction Stop
    if ($rootItem.Attributes -band [IO.FileAttributes]::ReparsePoint) {
        throw "Project root is a reparse point; refusing to follow it: $fullRoot"
    }
    $existingItems = @(Get-ChildItem -LiteralPath $fullRoot -Force -Recurse -ErrorAction Stop)
    $links = @($existingItems | Where-Object { $_.Attributes -band [IO.FileAttributes]::ReparsePoint })
    if ($links.Count -gt 0) {
        $linkNames = ($links | ForEach-Object { $_.FullName }) -join ', '
        throw "Reparse points require inspection before the runner can proceed: $linkNames"
    }
    return [pscustomobject]@{
        Path = $fullRoot
        ExistingItemCount = $existingItems.Count
        ReparsePointCount = $links.Count
        GitMetadataPresent = [bool](Test-Path -LiteralPath (Join-Path $fullRoot '.git'))
    }
}

function Invoke-CaptureText {
    param(
        [Parameter(Mandatory)]$Command,
        [string[]]$ArgumentList = @()
    )
    $output = (& $Command.Source @ArgumentList 2>&1 | Out-String).Trim()
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        throw "Command failed with exit code ${exitCode}: $($Command.Name) $($ArgumentList -join ' ')"
    }
    return $output
}

function Get-ExecutionPreflight {
    param(
        [Parameter(Mandatory)][string]$CanonicalProjectRoot,
        [switch]$ForDryRun
    )
    $python = Get-Command python -ErrorAction Stop
    $uv = Get-Command uv -ErrorAction Stop
    $codex = Get-Command codex -ErrorAction Stop
    $pwsh = Get-Command pwsh -ErrorAction Stop
    if ($PSVersionTable.PSVersion.Major -lt 7) {
        throw 'Run this controller with PowerShell 7 or later (pwsh) for bounded child-process control.'
    }
    $pythonVersion = Invoke-CaptureText -Command $python -ArgumentList @('--version')
    $uvVersion = Invoke-CaptureText -Command $uv -ArgumentList @('--version')
    $codexVersion = Invoke-CaptureText -Command $codex -ArgumentList @('--version')
    $pwshVersion = Invoke-CaptureText -Command $pwsh -ArgumentList @('--version')
    $topHelp = Invoke-CaptureText -Command $codex -ArgumentList @('--help')
    $execHelp = Invoke-CaptureText -Command $codex -ArgumentList @('exec', '--help')

    $requiredTopLevelFlags = @('--ask-for-approval')
    $requiredExecFlags = @('--cd', '--sandbox', 'workspace-write', '--skip-git-repo-check', '--output-last-message', '--ephemeral')
    foreach ($flag in $requiredTopLevelFlags) {
        if ($topHelp -notmatch [regex]::Escape($flag)) {
            throw "Installed Codex CLI does not support required top-level flag $flag."
        }
    }
    foreach ($flag in $requiredExecFlags) {
        if ($execHelp -notmatch [regex]::Escape($flag)) {
            throw "Installed Codex CLI does not support required exec flag/value $flag."
        }
    }

    $mcpList = Invoke-CaptureText -Command $codex -ArgumentList @('mcp', 'list')
    $linearRow = @($mcpList -split '\r?\n' | Where-Object { $_ -match '^\s*linear\s' } | Select-Object -First 1)
    $cliLinearAuthenticated = $false
    if ($linearRow.Count -gt 0) {
        $cliLinearAuthenticated = ($linearRow[0] -match '(?i)\benabled\b') -and
            ($linearRow[0] -notmatch '(?i)\bnot logged in\b|\bunsupported\b|\bdisabled\b') -and
            ($linearRow[0] -notmatch '(?i)\benabled\s+-\s*$')
    }
    if (-not $ForDryRun -and -not $cliLinearAuthenticated) {
        throw 'Codex CLI Linear MCP is not authenticated. Live execution fails closed; refresh CLI Linear access before starting a separate runner.'
    }

    return [pscustomobject]@{
        PythonPath = $python.Source
        PythonVersion = $pythonVersion
        UvPath = $uv.Source
        UvVersion = $uvVersion
        CodexPath = $codex.Source
        CodexVersion = $codexVersion
        PwshPath = $pwsh.Source
        PwshVersion = $pwshVersion
        CliLinearAuthenticated = $cliLinearAuthenticated
        LinearRow = if ($linearRow.Count -gt 0) { $linearRow[0].Trim() } else { 'missing' }
        Root = $CanonicalProjectRoot
    }
}

function Invoke-BoundedCodexProcess {
    param(
        [Parameter(Mandatory)][string]$PwshPath,
        [Parameter(Mandatory)][string]$CodexScriptPath,
        [Parameter(Mandatory)][string]$WorkingDirectory,
        [Parameter(Mandatory)][string[]]$Arguments,
        [Parameter(Mandatory)][string]$Mission,
        [Parameter(Mandatory)][DateTimeOffset]$DeadlineUtc
    )
    $startInfo = [Diagnostics.ProcessStartInfo]::new()
    $startInfo.FileName = $PwshPath
    $startInfo.WorkingDirectory = $WorkingDirectory
    $startInfo.UseShellExecute = $false
    $startInfo.CreateNoWindow = $false
    $startInfo.RedirectStandardInput = $true
    $startInfo.RedirectStandardOutput = $false
    $startInfo.RedirectStandardError = $false
    $startInfo.ArgumentList.Add('-NoProfile')
    $startInfo.ArgumentList.Add('-File')
    $startInfo.ArgumentList.Add($CodexScriptPath)
    foreach ($argument in $Arguments) {
        $startInfo.ArgumentList.Add([string]$argument)
    }

    $process = [Diagnostics.Process]::new()
    $process.StartInfo = $startInfo
    $timedOut = $false
    try {
        if (-not $process.Start()) {
            throw 'Codex CLI child process did not start.'
        }
        $process.StandardInput.WriteLine($Mission)
        $process.StandardInput.Close()
        while (-not $process.WaitForExit(250)) {
            if ([DateTimeOffset]::UtcNow -ge $DeadlineUtc) {
                if (-not $process.HasExited) {
                    $process.Kill($true)
                }
                $process.WaitForExit()
                $timedOut = $true
                break
            }
        }
        $process.WaitForExit()
        return [pscustomobject]@{
            ExitCode = if ($timedOut) { -2 } else { [int]$process.ExitCode }
            TimedOut = $timedOut
        }
    } finally {
        if (-not $process.HasExited) {
            $process.Kill($true)
            $process.WaitForExit()
        }
        $process.Dispose()
    }
}

function Assert-LinearSnapshot {
    param(
        [Parameter(Mandatory)][string]$Path,
        [Parameter(Mandatory)][string]$CanonicalProjectRoot
    )
    $fullPath = Get-FullPath -Path $Path
    if (-not (Test-PathInsideRoot -Path $fullPath -CanonicalProjectRoot $CanonicalProjectRoot)) {
        throw 'Linear snapshot must be stored inside the canonical project root.'
    }
    if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
        throw "Fresh Linear snapshot is required and was not found: $fullPath"
    }
    $snapshotItem = Get-Item -LiteralPath $fullPath -Force -ErrorAction Stop
    if ($snapshotItem.Attributes -band [IO.FileAttributes]::ReparsePoint) {
        throw 'Linear snapshot may not be a reparse point.'
    }
    try {
        $rawSnapshot = Get-Content -LiteralPath $fullPath -Raw -Encoding UTF8
        $snapshot = $rawSnapshot | ConvertFrom-Json -ErrorAction Stop
    } catch {
        throw "Linear snapshot is not valid JSON: $($_.Exception.Message)"
    }
    if ($snapshot.connected -ne $true) {
        throw 'Linear snapshot does not confirm an authenticated live Linear MCP read.'
    }
    if ($snapshot.project -ne $script:ProjectId) {
        throw "Linear snapshot project must be $script:ProjectId."
    }
    if ([string]::IsNullOrWhiteSpace([string]$snapshot.source)) {
        throw 'Linear snapshot must record the authenticated source.'
    }
    $checkedAtMatch = [regex]::Match($rawSnapshot, '"checkedAtUtc"\s*:\s*"([^"]+)"')
    if (-not $checkedAtMatch.Success) {
        throw 'Linear snapshot checkedAtUtc must be an ISO-8601 string.'
    }
    try {
        $checkedAt = [DateTimeOffset]::Parse($checkedAtMatch.Groups[1].Value, [Globalization.CultureInfo]::InvariantCulture)
    } catch {
        throw 'Linear snapshot checkedAtUtc must be an ISO-8601 timestamp.'
    }
    $age = [DateTimeOffset]::UtcNow - $checkedAt.ToUniversalTime()
    if ($age.TotalMinutes -gt 15 -or $age.TotalMinutes -lt -2) {
        throw 'Linear snapshot is stale or has a future timestamp; refresh it from Linear MCP.'
    }
    $snapshot.checkedAtUtc = $checkedAt.ToUniversalTime().ToString('o')
    $issues = @($snapshot.issues)
    if ($issues.Count -ne $script:Queue.Count) {
        throw 'Linear snapshot must contain exactly the five authorized queue issues.'
    }
    for ($i = 0; $i -lt $script:Queue.Count; $i++) {
        if ([string]$issues[$i].identifier -cne $script:Queue[$i]) {
            throw "Linear snapshot queue order mismatch at position $($i + 1)."
        }
        if ([string]$issues[$i].status -notin $script:AllowedIssueStates) {
            throw "Unsupported Linear status for $($issues[$i].identifier): $($issues[$i].status)"
        }
    }
    return $snapshot
}

function Assert-AuthorizedQueue {
    param([Parameter(Mandatory)][string[]]$IssueIds)
    if ($IssueIds.Count -ne $script:Queue.Count) {
        throw 'Issue list must contain exactly the five authorized I0 issue IDs.'
    }
    for ($i = 0; $i -lt $script:Queue.Count; $i++) {
        if ($IssueIds[$i] -cne $script:Queue[$i]) {
            throw "Issue allowlist/order mismatch at position $($i + 1)."
        }
    }
    return $true
}

function Get-PendingI0Issues {
    param(
        [Parameter(Mandatory)]$Snapshot,
        [Parameter(Mandatory)][System.Collections.IDictionary]$IssueResults
    )
    $pending = [Collections.Generic.List[string]]::new()
    foreach ($item in @($Snapshot.issues)) {
        $id = [string]$item.identifier
        $liveState = [string]$item.status
        $recorded = $IssueResults[$id]
        if ($liveState -in @('In Review', 'Done')) {
            continue
        }
        if ($null -ne $recorded -and $recorded.result -eq 'COMPLETED' -and $recorded.linearState -in @('In Review', 'Done')) {
            continue
        }
        if ($liveState -notin @('Backlog', 'Todo', 'In Progress')) {
            throw "Issue $id is in a non-runnable state: $liveState"
        }
        $pending.Add($id)
    }
    return @($pending.ToArray())
}

function Get-NextIssueAttempt {
    param([Parameter(Mandatory)][int]$PreviousAttempts)
    if ($PreviousAttempts -lt 0) {
        throw 'Previous attempt count cannot be negative.'
    }
    if ($PreviousAttempts -ge $script:MaxAttemptsPerIssue) {
        return $null
    }
    return $PreviousAttempts + 1
}

function ConvertFrom-WorkerResult {
    param(
        [Parameter(Mandatory)][AllowEmptyString()][string]$LastMessage,
        [Parameter(Mandatory)][int]$ProcessExitCode,
        [Parameter(Mandatory)][string]$ExpectedIssue
    )
    $fields = @{}
    $duplicates = [Collections.Generic.List[string]]::new()
    foreach ($line in ($LastMessage -split '\r?\n')) {
        if ($line -match '^([A-Z_]+)=(.*)$') {
            $key = $Matches[1]
            if ($fields.ContainsKey($key)) {
                $duplicates.Add($key)
            } else {
                $fields[$key] = $Matches[2].Trim()
            }
        }
    }
    $reason = $null
    if ($duplicates.Count -gt 0) {
        $reason = 'duplicate structured result fields'
    } elseif ($ProcessExitCode -ne 0) {
        $reason = "Codex process exited $ProcessExitCode"
    } elseif ($fields.OVERNIGHT_RESULT -cne 'COMPLETED') {
        $reason = 'worker did not report OVERNIGHT_RESULT=COMPLETED'
    } elseif ($fields.ISSUE_ID -cne $ExpectedIssue) {
        $reason = 'worker result issue ID does not match the active issue'
    } elseif ($fields.LINEAR_STATE -cne 'In Review') {
        $reason = 'issue was not moved to In Review'
    } elseif ($fields.TESTS_EXIT_CODE -cne '0' -or [string]::IsNullOrWhiteSpace([string]$fields.TESTS)) {
        $reason = 'fresh test command and zero exit code are required'
    } elseif ([string]::IsNullOrWhiteSpace([string]$fields.ARTIFACTS)) {
        $reason = 'reviewable artifact paths are required'
    }
    return [pscustomobject]@{
        Success = [string]::IsNullOrEmpty($reason)
        Result = if ([string]::IsNullOrEmpty($reason)) { 'COMPLETED' } else { 'BLOCKED' }
        IssueId = [string]$fields.ISSUE_ID
        LinearState = [string]$fields.LINEAR_STATE
        Tests = [string]$fields.TESTS
        TestExitCode = [string]$fields.TESTS_EXIT_CODE
        ArtifactPaths = @(([string]$fields.ARTIFACTS -split ';') | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
        StopReason = $reason
        Next = [string]$fields.NEXT
        ClaimCeiling = [string]$fields.CLAIM_CEILING
    }
}

function New-InitialState {
    param(
        [Parameter(Mandatory)][string]$Id,
        [Parameter(Mandatory)][double]$Hours,
        [Parameter(Mandatory)]$Preflight,
        [Parameter(Mandatory)]$Snapshot
    )
    return [pscustomobject]@{
        schemaVersion = 1
        sessionId = $Id
        project = $script:ProjectId
        root = $script:CanonicalRoot
        startedAtUtc = [DateTimeOffset]::UtcNow.ToString('o')
        endedAtUtc = $null
        maxElapsedHours = $Hours
        queue = @($script:Queue)
        preflight = [pscustomobject]@{
            pythonVersion = $Preflight.PythonVersion
            uvVersion = $Preflight.UvVersion
            codexVersion = $Preflight.CodexVersion
            pwshVersion = $Preflight.PwshVersion
            cliLinearAuthenticated = $Preflight.CliLinearAuthenticated
            linearSnapshotSource = [string]$Snapshot.source
            linearSnapshotCheckedAtUtc = [string]$Snapshot.checkedAtUtc
            rootExistingItemCount = $null
            reparsePointCount = 0
            gitMetadataPresent = $null
        }
        currentIssue = $null
        issueResults = [ordered]@{}
        result = 'RUNNING'
        next = $script:Queue[0]
        stopReason = $null
    }
}

function ConvertTo-StateHashtable {
    param([Parameter(Mandatory)]$ParsedState)
    $results = [ordered]@{}
    if ($null -ne $ParsedState.issueResults) {
        if ($ParsedState.issueResults -is [System.Collections.IDictionary]) {
            foreach ($key in $ParsedState.issueResults.Keys) {
                $results[[string]$key] = $ParsedState.issueResults[$key]
            }
        } else {
            foreach ($property in $ParsedState.issueResults.PSObject.Properties) {
                $results[$property.Name] = $property.Value
            }
        }
    }
    $state = [ordered]@{}
    foreach ($property in $ParsedState.PSObject.Properties) {
        if ($property.Name -ne 'issueResults') {
            $state[$property.Name] = $property.Value
        }
    }
    $state['issueResults'] = $results
    return $state
}

function Write-StateAtomic {
    param(
        [Parameter(Mandatory)]$State,
        [Parameter(Mandatory)][string]$Path
    )
    $json = ConvertTo-Json -InputObject $State -Depth 12
    $temporaryPath = Join-Path (Split-Path -Parent $Path) ('.state-' + [Guid]::NewGuid().ToString('N') + '.tmp')
    [IO.File]::WriteAllText($temporaryPath, $json + [Environment]::NewLine, [Text.UTF8Encoding]::new($false))
    if (Test-Path -LiteralPath $Path -PathType Leaf) {
        $backupPath = $temporaryPath + '.bak'
        [IO.File]::Replace($temporaryPath, $Path, $backupPath)
        [IO.File]::Delete($backupPath)
    } else {
        [IO.File]::Move($temporaryPath, $Path)
    }
}

function Get-SafeArtifactHashes {
    param(
        [Parameter(Mandatory)][string[]]$Paths,
        [Parameter(Mandatory)][string]$CanonicalProjectRoot
    )
    $hashes = [ordered]@{}
    foreach ($relativePath in $Paths) {
        if ([IO.Path]::IsPathRooted($relativePath)) {
            throw "Artifact path must be project-relative: $relativePath"
        }
        $fullPath = [IO.Path]::GetFullPath((Join-Path $CanonicalProjectRoot $relativePath))
        if (-not (Test-PathInsideRoot -Path $fullPath -CanonicalProjectRoot $CanonicalProjectRoot)) {
            throw "Artifact escaped the project root: $relativePath"
        }
        if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
            throw "Claimed artifact does not exist: $relativePath"
        }
        $item = Get-Item -LiteralPath $fullPath -Force -ErrorAction Stop
        if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) {
            throw "Artifact is a reparse point: $relativePath"
        }
        $hashes[$relativePath] = (Get-FileHash -LiteralPath $fullPath -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    return $hashes
}

function Format-CommandLine {
    param([Parameter(Mandatory)][string[]]$Arguments)
    $rendered = foreach ($argument in $Arguments) {
        if ($argument -match '[\s"]') {
            '"' + $argument.Replace('"', '\"') + '"'
        } else {
            $argument
        }
    }
    return ($rendered -join ' ')
}

function New-IssueMission {
    param(
        [Parameter(Mandatory)][string]$IssueId,
        [Parameter(Mandatory)][int]$Attempt,
        [Parameter(Mandatory)][int]$MaxRepairs,
        [Parameter(Mandatory)][string]$SessionDirectory
    )
    $repairInstruction = if ($Attempt -eq 1) {
        'Implement this issue from its live Linear requirements and acceptance criteria.'
    } else {
        "This is bounded repair attempt $($Attempt - 1) of $MaxRepairs. Diagnose the prior recorded failure and repair it without weakening acceptance criteria."
    }
    return @"
Work only on $IssueId in project P-RES-41, at D:\projects\BioSynGenBenchmark. Do not launch another Codex CLI process or the overnight controller.
Read the live issue and its latest instructions from Linear MCP. Verify its predecessor requirements and current state. Set this issue In Progress before work if it is not already started. $repairInstruction
Implement working software, run fresh tests and independent negative controls, inspect outputs, and record exact commands and evidence. Update this same issue to In Review only if every acceptance check passes. Never mark Done. Never create issues.
Stay within the authorized I0 contract: no Plate2bar or DynamisBench changes, historical or private corpora, participant data, locked/consumed holdouts, sibling evidence folder, cloud services, empirical thresholds, or claims of biological/device validation. Keep all output in the project root. The six-hour queue deadline and two-repair cap apply.
Before finishing, ensure reviewable artifacts exist and are project-relative. Return exactly one line for each field below, followed by a short readable summary:
OVERNIGHT_RESULT=COMPLETED|PARTIAL|BLOCKED
ISSUE_ID=$IssueId
LINEAR_STATE=In Review|In Progress|Todo
TESTS_EXIT_CODE=0|nonzero
TESTS=<exact test command and count/result>
ARTIFACTS=<semicolon-separated project-relative paths>
NEXT=<next authorized issue ID or STOP>
CLAIM_CEILING=NUMERICAL_VERIFICATION_ONLY;SIMULATED_MEASUREMENT_ONLY;BIOLOGICAL_VALIDATION=NOT_ESTABLISHED;REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED
STOP_REASON=<none or exact blocker>
Session record directory: $SessionDirectory
"@
}

function Invoke-Controller {
    if ($DryRun -and $Resume) {
        throw '-DryRun and -Resume cannot be combined.'
    }
    $rootInfo = Assert-ProjectRoot -Path $Root
    $fullRoot = $rootInfo.Path
    if ([string]::IsNullOrWhiteSpace($LinearSnapshotPath)) {
        throw 'A fresh Linear MCP issue snapshot is required. Pass -LinearSnapshotPath.'
    }
    $snapshot = Assert-LinearSnapshot -Path $LinearSnapshotPath -CanonicalProjectRoot $fullRoot
    Assert-AuthorizedQueue -IssueIds @($snapshot.issues | ForEach-Object { [string]$_.identifier }) | Out-Null
    $preflight = Get-ExecutionPreflight -CanonicalProjectRoot $fullRoot -ForDryRun:$DryRun

    if ([string]::IsNullOrWhiteSpace($SessionId)) {
        if ($Resume) {
            throw '-SessionId is required with -Resume.'
        }
        $SessionId = ([DateTimeOffset]::UtcNow.ToString('yyyyMMddTHHmmssZ') + '-' + [Guid]::NewGuid().ToString('N').Substring(0, 8))
    }
    if ($SessionId -notmatch '^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$') {
        throw 'SessionId may contain only letters, numbers, period, underscore, and hyphen.'
    }
    $runRoot = Join-Path $fullRoot 'runs\overnight'
    $sessionDirectory = Join-Path $runRoot $SessionId
    $statePath = Join-Path $sessionDirectory 'state.json'

    if ($Resume) {
        if (-not (Test-Path -LiteralPath $statePath -PathType Leaf)) {
            throw "No resumable state exists for session $SessionId."
        }
        $state = ConvertTo-StateHashtable -ParsedState (Get-Content -LiteralPath $statePath -Raw -Encoding UTF8 | ConvertFrom-Json -ErrorAction Stop)
        if ($state.project -ne $script:ProjectId -or $state.root -ine $fullRoot) {
            throw 'Resume state belongs to a different project or root.'
        }
        if (@($state.queue).Count -ne $script:Queue.Count) {
            throw 'Resume state does not match the authorized fixed issue count.'
        }
        Assert-AuthorizedQueue -IssueIds @($state.queue) | Out-Null
        $effectiveHours = [Math]::Min([double]$state.maxElapsedHours, $MaxHours)
        $state['maxElapsedHours'] = $effectiveHours
    } else {
        if (Test-Path -LiteralPath $sessionDirectory) {
            throw "Session directory already exists; use -Resume with its state instead: $sessionDirectory"
        }
        $runsDirectory = Join-Path $fullRoot 'runs'
        if (-not (Test-Path -LiteralPath $runsDirectory -PathType Container)) {
            New-Item -ItemType Directory -Path $runsDirectory -Force:$false | Out-Null
        }
        if (-not (Test-Path -LiteralPath $runRoot -PathType Container)) {
            New-Item -ItemType Directory -Path $runRoot -Force:$false | Out-Null
        }
        New-Item -ItemType Directory -Path $sessionDirectory -Force:$false | Out-Null
        $state = ConvertTo-StateHashtable -ParsedState (New-InitialState -Id $SessionId -Hours $MaxHours -Preflight $preflight -Snapshot $snapshot)
        $state.preflight.rootExistingItemCount = $rootInfo.ExistingItemCount
        $state.preflight.reparsePointCount = $rootInfo.ReparsePointCount
        $state.preflight.gitMetadataPresent = $rootInfo.GitMetadataPresent
    }

    $elapsed = [DateTimeOffset]::UtcNow - [DateTimeOffset]::Parse([string]$state.startedAtUtc)
    if ($elapsed.TotalHours -ge [double]$state.maxElapsedHours) {
        $state.result = 'BLOCKED'
        $state.stopReason = 'six-hour elapsed-time ceiling reached'
        $state.endedAtUtc = [DateTimeOffset]::UtcNow.ToString('o')
        Write-StateAtomic -State $state -Path $statePath
        throw $state.stopReason
    }

    $pending = Get-PendingI0Issues -Snapshot $snapshot -IssueResults $state.issueResults
    $state.next = if ($pending.Count -gt 0) { $pending[0] } else { $null }
    $state.stopReason = $null
    $state.result = 'RUNNING'
    $state.endedAtUtc = $null
    Write-StateAtomic -State $state -Path $statePath

    if ($DryRun) {
        $firstIssue = if ($pending.Count -gt 0) { $pending[0] } else { $script:Queue[0] }
        $resultPath = Join-Path $sessionDirectory ($firstIssue + '.attempt-1.last-message.txt')
        $arguments = @('--ask-for-approval', 'never', 'exec', '--cd', $fullRoot, '--sandbox', 'workspace-write', '--skip-git-repo-check', '--ephemeral', '--output-last-message', $resultPath, '-')
        $processArguments = @('-NoProfile', '-File', $preflight.CodexPath) + $arguments
        $display = '"' + $preflight.PwshPath + '" ' + (Format-CommandLine -Arguments $processArguments)
        $lines = @(
            'DRY RUN: command construction validated; Codex was not launched.'
            ('Root: ' + $fullRoot)
            ('Existing items inspected: ' + $rootInfo.ExistingItemCount)
            ('Reparse points: ' + $rootInfo.ReparsePointCount)
            ('Git metadata present: ' + $rootInfo.GitMetadataPresent)
            ('Python: ' + $preflight.PythonVersion)
            ('uv: ' + $preflight.UvVersion)
            ('Codex: ' + $preflight.CodexVersion)
            ('PowerShell: ' + $preflight.PwshVersion)
            ('Linear snapshot source: ' + $snapshot.source)
            ('Linear snapshot checked at: ' + $snapshot.checkedAtUtc)
            ('Codex CLI Linear authenticated: ' + $preflight.CliLinearAuthenticated)
            ('Queue: ' + ($script:Queue -join ' -> '))
            ('Pending: ' + ($pending -join ', '))
            ('Attempt cap: 1 initial + 2 repairs per issue')
            ('Elapsed-hour cap: ' + $state.maxElapsedHours)
            ('Constructed command: ' + $display)
            ('Process argument JSON: ' + (ConvertTo-Json -InputObject $processArguments -Compress))
            ('Session: ' + $SessionId)
            ('Resume command: .\scripts\overnight.ps1 -Root D:\projects\BioSynGenBenchmark -SessionId ' + $SessionId + ' -Resume -LinearSnapshotPath ' + $LinearSnapshotPath)
        )
        $transcriptPath = Join-Path $sessionDirectory 'dry-run.txt'
        [IO.File]::WriteAllLines($transcriptPath, $lines, [Text.UTF8Encoding]::new($false))
        $state.result = 'DRY_RUN'
        $state.endedAtUtc = [DateTimeOffset]::UtcNow.ToString('o')
        $state.next = if ($pending.Count -gt 0) { $pending[0] } else { $null }
        Write-StateAtomic -State $state -Path $statePath
        $lines | ForEach-Object { Write-Output $_ }
        Write-Output ('Transcript: ' + $transcriptPath)
        return
    }

    if (-not $preflight.CliLinearAuthenticated) {
        throw 'Live run stopped: Codex CLI Linear MCP is not authenticated.'
    }
    foreach ($issueId in $pending) {
        $prior = $state.issueResults[$issueId]
        $previousAttempts = if ($null -ne $prior) { [int]$prior.attempts } else { 0 }
        $attempt = Get-NextIssueAttempt -PreviousAttempts $previousAttempts
        $maxAttempts = $script:MaxAttemptsPerIssue
        if ($null -eq $attempt) {
            $state.result = 'BLOCKED'
            $state.stopReason = "two bounded repairs exhausted for $issueId"
            break
        }
        $issuePassed = $false

        while ($attempt -le $maxAttempts) {
            $elapsed = [DateTimeOffset]::UtcNow - [DateTimeOffset]::Parse([string]$state.startedAtUtc)
            if ($elapsed.TotalHours -ge [double]$state.maxElapsedHours) {
                $state.result = 'PARTIAL'
                $state.stopReason = 'six-hour elapsed-time ceiling reached'
                break
            }
            $prior = $state.issueResults[$issueId]
            $linearState = if ($null -ne $prior) {
                [string]$prior.linearState
            } else {
                [string](@($snapshot.issues | Where-Object { $_.identifier -eq $issueId } | Select-Object -First 1)[0].status)
            }
            $startedAt = if ($null -ne $prior -and $prior.startedAtUtc) {
                [string]$prior.startedAtUtc
            } else {
                [DateTimeOffset]::UtcNow.ToString('o')
            }
            $lastAttemptAt = [DateTimeOffset]::UtcNow.ToString('o')
            $lastMessagePath = Join-Path $sessionDirectory ($issueId + '.attempt-' + $attempt + '.last-message.txt')
            $state.currentIssue = $issueId
            $state.next = $issueId
            $state.issueResults[$issueId] = [pscustomobject]@{
                attempts = $attempt
                startedAtUtc = $startedAt
                lastAttemptAtUtc = $lastAttemptAt
                lastMessagePath = [IO.Path]::GetRelativePath($fullRoot, $lastMessagePath)
                endedAtUtc = $null
                result = 'RUNNING'
                linearState = $linearState
                tests = if ($null -ne $prior) { [string]$prior.tests } else { '' }
                artifactHashes = if ($null -ne $prior) { $prior.artifactHashes } else { [ordered]@{} }
                stopReason = $null
            }
            Write-StateAtomic -State $state -Path $statePath

            $arguments = @('--ask-for-approval', 'never', 'exec', '--cd', $fullRoot, '--sandbox', 'workspace-write', '--skip-git-repo-check', '--ephemeral', '--output-last-message', $lastMessagePath, '-')
            $mission = New-IssueMission -IssueId $issueId -Attempt $attempt -MaxRepairs 2 -SessionDirectory $sessionDirectory
            Write-Output "Starting $issueId attempt $attempt of $maxAttempts (foreground; Ctrl+C interrupts and leaves resumable state)."
            $deadlineUtc = [DateTimeOffset]::Parse([string]$state.startedAtUtc).AddHours([double]$state.maxElapsedHours)
            $workerProcess = Invoke-BoundedCodexProcess -PwshPath $preflight.PwshPath -CodexScriptPath $preflight.CodexPath -WorkingDirectory $fullRoot -Arguments $arguments -Mission $mission -DeadlineUtc $deadlineUtc
            $processExitCode = [int]$workerProcess.ExitCode
            $lastMessage = if (Test-Path -LiteralPath $lastMessagePath -PathType Leaf) { Get-Content -LiteralPath $lastMessagePath -Raw -Encoding UTF8 } else { '' }
            $outcome = ConvertFrom-WorkerResult -LastMessage $lastMessage -ProcessExitCode $processExitCode -ExpectedIssue $issueId
            if ($workerProcess.TimedOut) {
                $outcome.Success = $false
                $outcome.Result = 'BLOCKED'
                $outcome.StopReason = 'six-hour elapsed-time ceiling terminated the worker process'
            }
            $artifactHashes = [ordered]@{}
            if ($outcome.Success) {
                try {
                    $artifactHashes = Get-SafeArtifactHashes -Paths $outcome.ArtifactPaths -CanonicalProjectRoot $fullRoot
                } catch {
                    $outcome.Success = $false
                    $outcome.Result = 'BLOCKED'
                    $outcome.StopReason = $_.Exception.Message
                }
            }
            $state.issueResults[$issueId] = [pscustomobject]@{
                attempts = $attempt
                startedAtUtc = $startedAt
                lastAttemptAtUtc = $lastAttemptAt
                lastMessagePath = [IO.Path]::GetRelativePath($fullRoot, $lastMessagePath)
                endedAtUtc = [DateTimeOffset]::UtcNow.ToString('o')
                result = if ($outcome.Success) { 'COMPLETED' } else { 'FAILED' }
                linearState = $outcome.LinearState
                tests = $outcome.Tests
                testExitCode = $outcome.TestExitCode
                artifactHashes = $artifactHashes
                stopReason = $outcome.StopReason
                next = $outcome.Next
                claimCeiling = $outcome.ClaimCeiling
                processExitCode = $processExitCode
            }
            $state.currentIssue = $null
            if ($outcome.Success) {
                $issuePassed = $true
                $state.result = 'RUNNING'
                $state.stopReason = $null
                $nextIndex = [Array]::IndexOf($script:Queue, $issueId) + 1
                $state.next = if ($nextIndex -lt $script:Queue.Count) { $script:Queue[$nextIndex] } else { $null }
                Write-StateAtomic -State $state -Path $statePath
                Write-Output "$issueId passed its worker result gate and recorded artifact hashes."
                break
            }
            Write-StateAtomic -State $state -Path $statePath
            Write-Output "$issueId attempt $attempt did not pass: $($outcome.StopReason)"
            if ($workerProcess.TimedOut) {
                $state.result = 'PARTIAL'
                $state.stopReason = $outcome.StopReason
                break
            }
            if ($attempt -ge $maxAttempts) {
                $state.result = 'BLOCKED'
                $state.stopReason = "two bounded repairs exhausted for $issueId; $($outcome.StopReason)"
                break
            }
            $attempt++
        }
        if (-not $issuePassed) {
            break
        }
    }

    $state.endedAtUtc = [DateTimeOffset]::UtcNow.ToString('o')
    if ($state.result -eq 'RUNNING') {
        $remaining = Get-PendingI0Issues -Snapshot $snapshot -IssueResults $state.issueResults
        if ($remaining.Count -eq 0) {
            $state.result = 'COMPLETED'
            $state.next = $null
        } elseif ($null -eq $state.stopReason) {
            $state.result = 'PARTIAL'
            $state.next = $remaining[0]
            $state.stopReason = 'queue has remaining issues; resume from the recorded session'
        }
    }
    Write-StateAtomic -State $state -Path $statePath
    Write-Output ('Session result: ' + $state.result)
    Write-Output ('State: ' + $statePath)
    if ($state.stopReason) { Write-Output ('Stop reason: ' + $state.stopReason) }
}

if ($MyInvocation.InvocationName -ne '.') {
    Invoke-Controller
}
