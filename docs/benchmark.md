# I0 independent benchmark core

## Independence and context of use

The benchmark code lives in the separate biosyngen_benchmark package and imports only the Python standard library. Its input is a public measurement submission: value arrays, native timestamps, units, sign/frame, support coverage, and a declared clock contract. Numerical references are separate explicit inputs for known-answer software checks. The evaluator has no generator state, event tags, solver helpers, or Plate2bar imports.

An independent reference may establish a numerical implementation check. It does not establish biological realism, metrological fidelity, vendor equivalence, or real paired FORCE/LPT validation. I0 has no admitted empirical reference for those claims.

## Claim checks

- Force impulse balance integrates the public force series over an explicitly observed start/end interval. The reference supplies system mass, initial/final velocity, covered impulses from other external forces (including gravity), an external-reaction coverage flag, a qualified independent clock map, and a numerical tolerance. Missing interval endpoints, force samples, or incomplete external support return NOT_EVALUABLE.
- Ballistic flight checks public FORCE values at explicitly referenced flight times against the independent ideal-flight force. A nonzero incompatible value is rejected; absent times, clocks, or samples are not evaluable.
- Cable geometry independently calculates anchor-to-attachment length, projected cable rate, and angle from public anchor/attachment/velocity points. It matches by declared simulation time after applying the clock contract, never by array index.
- Bilateral FORCE claims require two declared foot channels, an explicit split assumption, and an independent numerical split reference. Supplying left/right arrays for a combined-only series is rejected. A combined total alone, or declared foot channels without a numerical reference, is NOT_EVALUABLE for a left/right claim.
- Biological and real paired FORCE/LPT claims remain NOT_EVALUABLE without the required independent reference; even a described reference does not turn this I0 prototype into an empirical validation procedure.

## Results and counts

Every requested claim receives PASS, REJECTED, or NOT_EVALUABLE with a reason, stable result code, and evidence ceiling. The report counts evaluated, eligible, admitted, rejected, and not-evaluable claims. Admitted means a narrow numerical check passed; it is not a universal credibility score. Rejection can falsify a narrow claim. NOT_EVALUABLE states what evidence is missing.

No thresholds here are empirical biomechanics thresholds. The included known-answer fixture derives its numerical tolerances from binary64 machine epsilon, signal scale, and the number of arithmetic operations.

Every numerical claim requires a qualified clock and an independent expected clock map. A candidate-declared clock without that reference is NOT_EVALUABLE; a clock that conflicts with it is REJECTED.
