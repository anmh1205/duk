# DUK mathematical model

This page documents implemented geometry and a parameterized **candidate planar
dynamics model**. It does not identify robot mass properties, select a controller,
or validate physical robot behavior. The static Gazebo preview remains separate.

## Coordinates, units and geometry

The source of established dimensions is
[`duk_leg_geometry.json`](../../robot/config/physical/duk_leg_geometry.json).
The numerical core uses millimetres and radians. Dynamics converts lengths to
metres once at its boundary and uses SI throughout.

In the canonical leg frame A=(0,0), +x is right, +y is up, and rotations are
counter-clockwise. D→A has fixed angle −30°. With q the directed angle AB→DA:

$$
\theta_{AB}=\theta_{DA}-q,\qquad
D=-AD[\cos\theta_{DA},\sin\theta_{DA}]^T,\qquad
B=AB[\cos\theta_{AB},\sin\theta_{AB}]^T.
$$

C is the configured physical intersection of circles (B, BC) and (D, CD).
With e=(C−B)/BC and N the 90° CCW operator:

$$
H=B-BH\,e,\qquad O=H+s_O OH\,Ne.
$$

The configured carrier side determines s_O. B, C and O are one rigid carrier;
H is a construction point, not another joint. Both legs use the same solver and
frame without mirroring. The sagittal model maps canonical y to world z.

## Velocity, acceleration and inverse position

The loop equations are |C−B|²=BC² and |C−D|²=CD². Writing r=C−B, s=C−D,
their first and second q derivatives give:

$$
\begin{bmatrix}r^T\\s^T\end{bmatrix} C'
=\begin{bmatrix}r^T B'\\0\end{bmatrix},\qquad
\begin{bmatrix}r^T\\s^T\end{bmatrix} C''
=\begin{bmatrix}r^TB''-\|C'-B'\|^2\\-\|C'\|^2\end{bmatrix}.
$$

B''=−(B−A). H/O derivatives follow the same affine rigid-carrier map at each
order. For any moving point p:

$$
\dot p=p'(q)\dot q,\qquad
\ddot p=p'(q)\ddot q+p''(q)\dot q^2.
$$

`leg_second_derivative` returns mm/rad²; `point_acceleration` returns mm/s².
The loop Jacobian is singular at circle tangency; those configurations are
rejected for derivatives. Numerical tolerances are not physical operating limits.

For inverse wheel position, BO is invariant. A candidate B lies on circles
(A, AB) and (O_target, BO). Each candidate q=θ_DA−atan2(B_y,B_x) is sent through
the existing forward solver and checked against O_target and physical topology.
`inverse_wheel_position` returns all matching principal-angle solutions or raises
an unreachable/singularity error. One leg DOF cannot independently prescribe two
arbitrary wheel coordinates; the implementation does not project an invalid
target onto the trajectory.

## Candidate planar biped: seven coordinates

`build_planar_biped_model` uses the following **mathematical coordinates**:

$$
\eta=[x,z,\theta,q_L,q_R,\phi_L,\phi_R]^T.
$$

(x,z) is a body-frame reference origin in the world; θ is body CCW pitch in the
(x,z) plane. q_L/q_R retain the canonical linkage definition. φ_L/φ_R are
**absolute** CCW wheel angles, not motor/encoder coordinates. Roll, yaw and
lateral dynamics are excluded. The preview's arbitrary body box is not used to
infer a mass, inertia, CoM or physical mount offset.

Each side has an unknown body-frame A mount offset a_i. Its solved local point
r_i(q_i) maps to the world as:

$$
p_i=[x,z]^T+R(\theta)(a_i+r_i(q_i)).
$$

The body CoM is a separate unknown offset from (x,z). Each AB, CD and BCO link
has independent unknown CoM coordinates (u,n) along its own unit direction and
left normal. AB uses A→B, CD uses D→C, BCO uses B→C. No midpoint or uniform-link
assumption is made. Wheel CoMs are assumed concentric at O for this candidate
model; eccentric wheels would require additional parameters and equations.

There are nine modeled rigid bodies: the torso plus four on each side (AB, CD,
BCO, wheel). AD and the fixed mounts are included in the torso's unidentified
aggregate properties; they are not counted as extra moving links.

## Unknown quantities stay symbolic

The [parameter template](../../robot/config/physical/duk_planar_parameters.example.json)
contains 43 named entries, all `null`. Null means **unknown**, never zero.

| Names | Meaning | Units |
| --- | --- | --- |
| `g_m_s2` | gravity magnitude, directed downward | m/s² |
| `m_body_kg`, `m_{AB,CD,BCO,wheel}_{L,R}_kg` | individual body masses | kg |
| `I_body_kg_m2`, `I_{AB,CD,BCO,wheel}_{L,R}_kg_m2` | planar inertia about each CoM | kg·m² |
| `c_body_x_m`, `c_body_z_m` | torso CoM in the body reference frame | m |
| `c_{AB,CD,BCO}_{L,R}_{u,n}_m` | moving-link CoM in its link frame | m |
| `a_{L,R}_{x,z}_m` | A mount offset in the body frame | m |
| `b_q_{L,R}_N_m_s` | viscous damping conjugate to linkage q | N·m·s/rad |
| `b_w_{L,R}_N_m_s` | wheel/carrier relative viscous damping | N·m·s/rad |
| `mu_{L,R}` | Coulomb friction coefficient | dimensionless |

Mass, inertia and gravity are positive; damping/friction may be zero. CoM and
mount offsets are signed. These assumptions constrain variables but do not assign
measurements. Motor calibration, gearing, friction beyond the explicit viscous/
Coulomb candidate laws, actuator limits and control gains are not guessed.

## Equations of motion and reaction torque

For each body j let J_j be its world CoM Jacobian and w_j its absolute planar
angular-velocity row. Then:

$$
M=\sum_j(m_jJ_j^TJ_j+I_jw_j^Tw_j),\quad
V=\sum_jm_jgz_j,\quad G=\nabla_\eta V,
$$

$$
h=\sum_j\left(m_jJ_j^T\dot J_j\dot\eta
+I_jw_j^T\dot w_j\dot\eta\right).
$$

This gives the same convective force as differentiating the Lagrangian T−V,
with T=½η̇ᵀMη̇. The implementation retains symbolic quantities rather than
expanding pages of repeated radicals and trigonometric expressions.

The full residual is:

$$
M\ddot\eta+h+G+D\dot\eta-Bu-J_c^T\lambda=0.
$$

u=[Q_qL,Q_qR,τ_wL,τ_wR] contains **generalized linkage forces**, not calibrated
leg motor torques, and ideal wheel/carrier relative torques. If β_i is the BCO
carrier's local CCW angle, its absolute angular speed is θ̇+β_i' q̇_i.
The wheel's relative spin row s_i gives:

$$
\omega_{rel,i}=\dot\phi_i-\dot\theta-\beta_i'\dot q_i=s_i\dot\eta.
$$

Wheel input columns in B are s_iᵀ, so wheel torque also produces equal/opposite
reaction on body pitch and carrier motion. D includes b_w s_iᵀs_i and the q
viscous terms. Applying only a wheel-spin torque and forgetting this reaction
would violate virtual work. Mapping Q_q to actual motors awaits identification
of their actuation geometry/transmission and encoder sign.

## Optional ground-contact mode

For flat ground at z=0, normal gaps are γ_i=z_Oi−r_w. The normal Jacobian contains
∂z_Oi/∂η. For positive CCW absolute wheel rotation, bottom-point horizontal speed
is ẋ_Oi+r_w φ̇_i, giving the rolling rows:

$$
J_{roll,i}=\partial x_{Oi}/\partial\eta+r_w e_{\phi_i}^T.
$$

J_c stacks both normal rows then both rolling rows. λ=[N_L,N_R,T_L,T_R] uses
positive upward normal force and positive rightward tangential force. For an
explicitly selected sticking-contact mode:

$$
J_c\dot\eta=0,\qquad J_c\ddot\eta+\dot J_c\dot\eta=0.
$$

`contact_acceleration_bias` supplies the last term. `contact_conditions` exposes
γ≥0, N≥0, γN=0 and |T|≤μN. The model does not pick an active set, invert a
potentially rank-deficient contact system, or solve sliding, liftoff or impact.
Symmetric sagittal wheel projections can produce dependent constraints; check
rank before solving any selected mode. A friction bound alone is not a complete
sliding-friction or impact law.

## Use and verification

Install `duk-core[symbolic]` or use the existing research/simulation image.
The base numerical geometry import itself does not require SymPy. From repo root:

```bash
python robot/software/core/examples/duk_planar_model.py
mkdir -p .local
cp robot/config/physical/duk_planar_parameters.example.json .local/planar-parameters.json
python robot/software/core/examples/duk_planar_model.py --parameters .local/planar-parameters.json
```

Replace only quantities actually
identified. Missing/null entries remain symbolic. `bind_parameters` validates
partial values; `bind_configuration` validates both physical assembly branches
and nonsingular SI coordinates before numerical substitution. Raw SymPy `.subs`
is available but bypasses those guards. Symbolic expressions apply only on that
admissible branch, not at singular boundaries or negative square-root domains.

Binding returns a substitution map and does not mutate the model. For example,
with `geometry` loaded from the canonical JSON and `identified_values` containing
only the non-null entries from the local parameter file:

```python
from duk_core.modeling import build_planar_biped_model

model = build_planar_biped_model(geometry)
bindings = model.bind_parameters(identified_values)
partial_mass_matrix = model.mass_matrix.subs(bindings)
# Every quantity not identified remains a symbol in partial_mass_matrix.
```

Tests compare both symbolic leg geometries against the canonical solver, second
derivatives against independent finite differences, inverse/FK round trips,
M symmetry/positive definiteness with **synthetic test-only** data, G against the
potential gradient, h against independent Lagrange/Christoffel evaluation,
contact acceleration against finite differences, and damping/reaction signs.
Synthetic fixtures do not enter the parameter template or Gazebo.

The Lagrangian and constrained-force formulation follows standard multibody
mechanics; see [MIT Underactuated Robotics: multibody dynamics](https://underactuated.mit.edu/multibody.html).
The implementation uses [SymPy](https://docs.sympy.org/latest/modules/physics/mechanics/lagrange.html)
for symbolic algebra. These sources establish methods, not DUK physical data.

The Gazebo robot remains a static rendering test until mass/actuator/contact
properties and dynamics integration are independently established. No controller
or physical-platform capability is selected by these equations.

## Grounded visualization frames

A=(0,0) is a body-frame convention, not a world constraint. The planar notebook
placement is z_A = R − O_local.y, giving z_O=R and a tire tangent to ground z=0.
It translates the entire mechanism, so all link/carrier invariants remain intact.

The two-leg RViz preview additionally solves lateral body roll when qL and qR
differ; its finite-width cylindrical tires are both tangent to the ground. See
the [grounded preview](../development/kinematic-bench.md) for the geometric
constraints and frame labels. This 3D visualization placement is **not** a new
roll dynamics model: the candidate 7-DOF dynamics above remains sagittal. Contact
forces, impacts, tire compliance and no-slip rolling have not been established.
