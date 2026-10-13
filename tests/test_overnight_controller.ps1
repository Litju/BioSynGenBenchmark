$ErrorActionPreference = 'Stop'
$controllerPath = Join-Path $PSScriptRoot '..\scripts\overnight.ps1'
. $controllerPath

$script:Assertions = 0

function Assert-True {
    param([bool]$Condition, [string]$Message)
    $script:Assertions++
    if (-not $Condition) { throw "FAIL: $Message" }
}

function Assert-Throws {
    param([scriptblock]$Action, [string]$Message)
    $script:Assertions++
    try {
        & $Action
    } catch {
        return
    }
    throw "FAIL: expected failure: $Message"
}

$canonicalQueue = @('RES-475', 'RES-476', 'RES-477', 'RES-478', 'RES-479')
Assert-True (Assert-AuthorizedQueue -IssueIds $canonicalQueue) 'authorized queue is accepted'
Assert-Throws { Assert-AuthorizedQueue -IssueIds @('RES-476', 'RES-475', 'RES-477', 'RES-478', 'RES-479') } 'queue ordering is enforced'
Assert-Throws { Assert-AuthorizedQueue -IssueIds @('RES-475', 'RES-476', 'RES-477', 'RES-478', 'RES-480') } 'unapproved issue is rejected'
Assert-Throws { Assert-ProjectRoot -Path 'D:\projects\BioSynGenBenchmark\missing-root' } 'nonexistent or noncanonical root fails closed'
Assert-Throws { Assert-LinearSnapshot -Path 'D:\projects\BioSynGenBenchmark\missing-linear-snapshot.json' -CanonicalProjectRoot 'D:\projects\BioSynGenBenchmark' } 'missing Linear authority fails closed'
$mission = New-IssueMission -IssueId 'RES-475' -Attempt 1 -MaxRepairs 2 -SessionDirectory 'runs\overnight\fixture'
Assert-True ($mission -match 'Read the live issue' -and $mission -match 'Do not launch another Codex CLI process') 'worker reads current Linear authority without recursive launch'
$repairMission = New-IssueMission -IssueId 'RES-475' -Attempt 3 -MaxRepairs 2 -SessionDirectory 'runs\overnight\fixture'
Assert-True ($repairMission -match 'repair attempt 2 of 2') 'repair prompt is finite and capped'
Assert-True ((Get-NextIssueAttempt -PreviousAttempts 0) -eq 1 -and (Get-NextIssueAttempt -PreviousAttempts 2) -eq 3) 'issue receives one initial attempt and two repairs'
Assert-True ($null -eq (Get-NextIssueAttempt -PreviousAttempts 3)) 'exhausted issue cannot be retried again'
$emptyIssueState = ConvertTo-StateHashtable -ParsedState ([pscustomobject]@{ schemaVersion = 1; issueResults = [ordered]@{} })
Assert-True ($emptyIssueState.issueResults.Count -eq 0) 'empty issue map survives initial-state normalization'

$fakeRoot = 'D:\projects\BioSynGenBenchmark'
$fakeSession = Join-Path $fakeRoot ('runs\overnight\controller-test-' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $fakeSession -Force:$false | Out-Null
try {
    $fakeLog = Join-Path $fakeSession 'fake-worker-result.log'
    $validMessage = @'
OVERNIGHT_RESULT=COMPLETED
ISSUE_ID=RES-475
LINEAR_STATE=In Review
TESTS_EXIT_CODE=0
TESTS=locally generated fake result record; parser fixture only
ARTIFACTS=AGENTS.md;scripts/overnight.ps1;README.md
NEXT=RES-476
CLAIM_CEILING=NUMERICAL_VERIFICATION_ONLY;SIMULATED_MEASUREMENT_ONLY
STOP_REASON=none
'@
    [IO.File]::WriteAllText($fakeLog, $validMessage, [Text.UTF8Encoding]::new($false))
    $valid = ConvertFrom-WorkerResult -LastMessage (Get-Content -LiteralPath $fakeLog -Raw) -ProcessExitCode 0 -ExpectedIssue 'RES-475'
    Assert-True $valid.Success 'complete structured result with exit code zero is accepted'
    Assert-True ($valid.Next -eq 'RES-476') 'next issue is read from the structured result'

    $nonzero = ConvertFrom-WorkerResult -LastMessage (Get-Content -LiteralPath $fakeLog -Raw) -ProcessExitCode 7 -ExpectedIssue 'RES-475'
    Assert-True (-not $nonzero.Success -and $nonzero.StopReason -match 'exited 7') 'nonzero process exit fails closed'
    $wrongIssue = ConvertFrom-WorkerResult -LastMessage (Get-Content -LiteralPath $fakeLog -Raw) -ProcessExitCode 0 -ExpectedIssue 'RES-476'
    Assert-True (-not $wrongIssue.Success -and $wrongIssue.StopReason -match 'issue ID') 'mismatched issue result fails closed'
    $duplicateFields = $validMessage + [Environment]::NewLine + 'ISSUE_ID=RES-476'
    $duplicate = ConvertFrom-WorkerResult -LastMessage $duplicateFields -ProcessExitCode 0 -ExpectedIssue 'RES-475'
    Assert-True (-not $duplicate.Success -and $duplicate.StopReason -match 'duplicate') 'duplicate structured fields fail closed'
    $blockedMessage = $validMessage.Replace('LINEAR_STATE=In Review', 'LINEAR_STATE=In Progress').Replace('OVERNIGHT_RESULT=COMPLETED', 'OVERNIGHT_RESULT=BLOCKED')
    [IO.File]::WriteAllText($fakeLog, $blockedMessage, [Text.UTF8Encoding]::new($false))
    $blocked = ConvertFrom-WorkerResult -LastMessage (Get-Content -LiteralPath $fakeLog -Raw) -ProcessExitCode 0 -ExpectedIssue 'RES-475'
    Assert-True (-not $blocked.Success -and $blocked.StopReason -match 'COMPLETED') 'blocked worker log stops the queue'
    $missingTests = $validMessage.Replace('TESTS_EXIT_CODE=0', 'TESTS_EXIT_CODE=1')
    $failedTests = ConvertFrom-WorkerResult -LastMessage $missingTests -ProcessExitCode 0 -ExpectedIssue 'RES-475'
    Assert-True (-not $failedTests.Success -and $failedTests.StopReason -match 'test command') 'failed tests stop the queue'

    $snapshot = [pscustomobject]@{
        issues = @(
            [pscustomobject]@{ identifier = 'RES-475'; status = 'In Review' }
            [pscustomobject]@{ identifier = 'RES-476'; status = 'Todo' }
            [pscustomobject]@{ identifier = 'RES-477'; status = 'Todo' }
            [pscustomobject]@{ identifier = 'RES-478'; status = 'Todo' }
            [pscustomobject]@{ identifier = 'RES-479'; status = 'Todo' }
        )
    }
    $pending = Get-PendingI0Issues -Snapshot $snapshot -IssueResults ([ordered]@{})
    Assert-True ($pending.Count -eq 4 -and $pending[0] -eq 'RES-476') 'Linear In Review issue is not executed again'
    $snapshot.issues[1].status = 'Canceled'
    Assert-Throws { Get-PendingI0Issues -Snapshot $snapshot -IssueResults ([ordered]@{}) } 'canceled issue stops execution'

    $statePath = Join-Path $fakeSession 'atomic-state.json'
    $state = [ordered]@{ schemaVersion = 1; sessionId = 'controller-test'; issueResults = [ordered]@{} }
    Write-StateAtomic -State $state -Path $statePath
    $state.issueResults['RES-475'] = [pscustomobject]@{ result = 'COMPLETED' }
    $state.currentIssue = 'RES-476'
    $state.result = 'RUNNING'
    $state.issueResults['RES-476'] = [pscustomobject]@{ attempts = 1; result = 'RUNNING'; lastAttemptAtUtc = [DateTimeOffset]::UtcNow.ToString('o') }
    Write-StateAtomic -State $state -Path $statePath
    $readback = Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json
    Assert-True ($readback.issueResults.'RES-475'.result -eq 'COMPLETED') 'atomic state replacement persists a completed issue'
    Assert-True ($readback.currentIssue -eq 'RES-476' -and $readback.issueResults.'RES-476'.result -eq 'RUNNING') 'interruption leaves the active issue resumable in durable state'

    $fakeWorkerPath = Join-Path $fakeSession 'fake-codex.ps1'
    $fakeWorkerContent = 'Write-Output ''FAKE_WORKER_STARTED''' + [Environment]::NewLine + '[Console]::In.ReadToEnd() | Out-Null' + [Environment]::NewLine + 'exit 0'
    [IO.File]::WriteAllText($fakeWorkerPath, $fakeWorkerContent, [Text.UTF8Encoding]::new($false))
    $pwshPath = (Get-Command pwsh -ErrorAction Stop).Source
    $child = Invoke-BoundedCodexProcess -PwshPath $pwshPath -CodexScriptPath $fakeWorkerPath -WorkingDirectory $fakeSession -Arguments @('fixture') -Mission 'synthetic worker prompt' -DeadlineUtc ([DateTimeOffset]::UtcNow.AddSeconds(10))
    Assert-True ($child.ExitCode -eq 0 -and -not $child.TimedOut) 'bounded foreground worker returns its real exit code'

    $fakeSleeperPath = Join-Path $fakeSession 'fake-sleeper.ps1'
    [IO.File]::WriteAllText($fakeSleeperPath, 'Start-Sleep -Seconds 5', [Text.UTF8Encoding]::new($false))
    $timedChild = Invoke-BoundedCodexProcess -PwshPath $pwshPath -CodexScriptPath $fakeSleeperPath -WorkingDirectory $fakeSession -Arguments @('fixture') -Mission 'synthetic timeout prompt' -DeadlineUtc ([DateTimeOffset]::UtcNow.AddMilliseconds(1000))
    Assert-True ($timedChild.TimedOut -and $timedChild.ExitCode -eq -2) 'elapsed-time deadline terminates a stalled worker'
} finally {
    $resolvedFakeSession = [IO.Path]::GetFullPath($fakeSession)
    $resolvedRoot = [IO.Path]::GetFullPath($fakeRoot).TrimEnd('\') + '\'
    if (-not $resolvedFakeSession.StartsWith($resolvedRoot, [StringComparison]::OrdinalIgnoreCase)) {
        throw 'Refusing to remove a test directory outside the project root.'
    }
    Remove-Item -LiteralPath $resolvedFakeSession -Recurse -Force
}

Write-Output "PASS: $script:Assertions controller assertions passed."
