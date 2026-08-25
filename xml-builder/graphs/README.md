# Graphs folder

Two things live here:

1. **Sanity-check tooling** (`mass_report.py`, `mass_vs_length.py`) that confirms
   the lambda-derived mass model is *implemented* correctly.
2. This README, which documents the separate question of *how the lambda
   values themselves were picked* -- `mass_report.py` can't answer that,
   because by construction `builder.py`'s `_linear_density` makes
   `mass = LAMBDA * length` exactly, for whatever `LAMBDA` you hand it. The
   scripts verify the plumbing, not the choice of constant.

## What LAMBDA_* is for

`builder.py` needs a single density constant per link (`THIGH_DENSITY`,
`SHANK_DENSITY`, `FOOT_DENSITY`) so that MuJoCo's `density x volume`
auto-mass reduces to a pure proportional model:

    M_link = LAMBDA_link * length

with **no intercept** -- see `LAMBDA_THIGH`/`LAMBDA_SHANK`/`LAMBDA_FOOT` in
`constant_params.json` and `_linear_density` in `builder.py`. This has to be
a pure proportionality because the generator has to support arbitrary
`thigh_length`/`shank_length`/`foot_length` chosen anywhere in their design
ranges (see `DESIGN_PARAMETERS.md`), and a MuJoCo geom density can only scale
mass linearly with volume -- it can't add a fixed offset.

## Why that's a problem, and the fix

A real link's mass doesn't actually follow `M = lambda * L` with zero
intercept. It follows something closer to

    M = m * L + c

where `m` (kg/m) is the material's mass rate as the link gets longer, and
`c` (kg) is the roughly length-independent mass of the hardware riding on
that link (bearings, mounts, wiring, etc.) that doesn't shrink or grow with
`L`. Forcing that through the origin by just using `m` as the constant would
systematically *under*-predict mass everywhere in the design's actual length
range, because it throws away `c` entirely.

Instead, `m` and `c` were fit to the best-available reference mass/length
data for each link, and then converted into the single proportional constant
`builder.py` needs via **least-squares regression through the origin**,
minimizing `sum((y_i - M*x_i)^2)` over the link's expected length range
`x in [a, b]`:

    M = m + c * [(b^2 - a^2)/2] / [(b^3 - a^3)/3]

(discrete-point form: `M = m + c * sum(x_i) / sum(x_i^2)`). This is just
ordinary least squares with the intercept term dropped, evaluated over the
continuous interval `[a, b]` instead of discrete samples. `[a, b]` was taken
as `[0.15, 0.5]` m, a rounded interval covering both `thigh_length` and
`shank_length`'s ranges from `DESIGN_PARAMETERS.md` with margin, so `M`
minimizes error across the whole span the parametric generator can actually
produce -- not just at one reference length.

### The two fits used

| Link | m (kg/m) | c (kg) | a, b (m) | M = LAMBDA | Constant used |
|---|---|---|---|---|---|
| Thigh | 2.95 | 0.105 | 0.15, 0.5 | 3.2446 | `LAMBDA_THIGH = 3.24` |
| Shank / Foot | 0.825 | 0.2325 | 0.15, 0.5 | 1.4773 | `LAMBDA_SHANK = LAMBDA_FOOT = 1.48` |

Both rounded values match `constant_params.json` exactly, confirming this is
the method that produced the current constants.

### Intuition

- If `c = 0`, `M = m` exactly -- no correction needed, the real line already
  passes through the origin.
- The correction grows with how far `[a, b]` sits from being centered at
  zero, and with the ratio `c/L` -- an interval like `[0.15, 0.5]` (all
  positive, nowhere near zero) pulls `M` up well above `m` to compensate for
  the missing intercept, which is why both constants above land noticeably
  higher than their respective `m`.
- On a symmetric interval `[-a, a]` the intercept cancels out and `M = m`;
  on `[0, L]` it simplifies to `M = m + 3c / (2L)`.

## Changing a lambda value

If `m`, `c`, or the design's length range change, recompute `M` with the
formula above (or the discrete-point version, if fitting straight from
reference data points), update the corresponding constant in
`constant_params.json`, then re-run `mass_report.py` and
`mass_vs_length.py` to confirm the built model still produces
`mass = LAMBDA * length` as expected -- those scripts catch plumbing bugs
in `_linear_density`, not a bad choice of `m`/`c`/`[a, b]`.
