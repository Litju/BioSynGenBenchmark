# Security and sensitive-data handling

This repository is a local numerical research prototype with no released supported-version series yet. It does not require sensor accounts, participant records or production secrets to run.

Do not place credentials, participant identifiers, proprietary files or restricted benchmark content in issues, pull requests, logs or test fixtures. Use synthetic minimal reproductions. A physics or evaluator false acceptance can be a scientific integrity defect even when it is not a conventional security exploit.

For a sensitive vulnerability, use the repository's private vulnerability reporting option if available. If it is unavailable, request a private contact channel with a short non-sensitive issue; do not post exploit details or confidential bytes. This policy does not assert that a private-reporting feature has been enabled or promise a response SLA.

Maintainers should preserve bounded evidence locally, assess the affected claims/versions, reproduce the defect independently, and publish a scoped fix and limitations after content review. Scan the full intended Git history before publication. Removing a secret from the latest file does not remove it from older commits; coordinate any credential revocation privately and retain no active secret in public history.
