# AI Use Note — HW1 Part 2 (MuJoCo Rotation Sandbox)

### Task 1 — `01_rotation_sandbox.py` (current-frame vs. fixed-frame composition)

**What I asked:**
> I asked the AI to help me understand and implement the `compose_sequence` function for elemental rotations about the current/body frame and fixed/space frame. I also asked for help debugging Python errors related to indentation and variables, and for guidance on extending the provided MuJoCo starter code to animate the rotations smoothly.

**How I used the output:**
> I used the AI-generated code and explanations as a reference to complete the starter code. I tested the implementation in MuJoCo using the same sequence of rotations for both current-frame and fixed-frame composition and verified that the final orientations were visibly different.

---

### Task 2 — `02_verify_skew_properties.py` / `02_verify_skew_properties_live.py` (numerical verification)

**What I asked:**
> I asked the AI to help implement the numerical checks for the identities \(R(v \times w) = (Rv) \times (Rw)\) and \(R\hat{\omega}R^T = \widehat{R\omega}\). I also asked for help creating an enhanced version with live MuJoCo visualization and automatic residual plot generation.

**How I used the output:**
> I used the generated code to complete the numerical verification functions and ran the scripts independently. I checked the resulting residuals, which were approximately \(10^{-15}\), consistent with floating-point machine precision. I also tested the live visualization and plot generation.

---

### Other AI use (tooling, debugging, README, etc.)

**What I asked:**
> I asked the AI for help with Python debugging, setting up and running the MuJoCo project on Windows, organizing the GitHub repository, creating a `.gitignore`, updating the README, and preparing the submission files.

**How I used the output:**
> I reviewed the suggestions and applied them to set up, test, organize, and document the project.

