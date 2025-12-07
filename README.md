/sp.constitution

Project: Physical AI & Humanoid Robotics – AI-Native Textbook (Project 1)
(AI/Spec-Driven Book Creation using Docusaurus, GitHub Pages or Vercel, and Spec-Kit Plus)

Core principles:

* Spec-driven authoring: No chapter written without a prior approved specification
* AI-augmented but human-verified content (Claude Code + human-in-the-loop review)
* Ground truth anchored in official ROS 2, NVIDIA Isaac, Gazebo, and Unity documentation
* Progressive pedagogy: beginner → intermediate → capstone autonomy
* Open educational resource (OER) compliant: reusable, remixable, redistributable
* Automation-first publishing: CI/CD-ready Docusaurus deployment to GitHub Pages or Vercel

Key standards:

* Every chapter must stem from a `/sp.specify`-approved design
* All technical assertions validated against trusted sources (ROS docs, Isaac SDK, etc.)
* Code snippets must be executable in a standard Ubuntu 22.04 + ROS 2 Humble environment
* Markdown-only content with Docusaurus frontmatter (title, description, slug, sidebar)
* Repository must include:
  - Clean branch strategy (main + feature branches)
  - Semantic commit messages
  - Public GitHub Issues for community feedback
* Writing must follow: Concept → Visual → Code → Lab → Reflection
* AI output must be edited for clarity, safety, and pedagogical effectiveness

Constraints:

* Output format: Static site via Docusaurus v3+
* Hosting: GitHub Pages or Vercel only (no Vercel/Netlify for base submission)
* Source control: Public GitHub repository
* AI stack: Spec-Kit Plus + Claude Code only (no external LLM APIs in draft phase)
* Content pipeline: `/sp.specify` → AI draft → human edit → test → GitHub Pages or Vercel deploy
* All content must be original, non-infringing, and free of hallucinated APIs or libraries
* No prose generated without explicit spec alignment
* SEO-optimized headings, meta descriptions, and internal linking required

Success criteria:

* Live book deployed at `<username>.github.io/<repo>`
* Zero broken links or image 404s
* All labs verified in simulation (Gazebo/Isaac Sim)
* Docusaurus builds cleanly with no warnings in production mode
* Repository serves as a template for future Panaversity AI-native books
* Learner can complete the capstone project using only this book
________________________________________________

/sp.specify  High-Level Book Architecture – Physical AI & Humanoid Robotics (Project 1)

Book Title:  
Physical AI & Humanoid Robotics: From Simulation to Embodied Intelligence

Primary Objective:  
Enable self-learners to build and command a simulated autonomous humanoid robot using ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action (VLA) pipelines—all through an AI-native, spec-first textbook.

Pedagogical Sequence:  
Spec → Sim → Code → AI → Integrate → Deploy (simulation-first, hardware-optional)

---

[Modules 1–4 and Infrastructure section remain identical to your provided specimen, so they are omitted here for brevity but are fully retained in semantic intent.]
 
---

Scope Control Note:  
This constitution governs **Project 1 only**: the AI/spec-driven textbook.  
A second constitution (`/sp.constitution` for Project 2) will define the RAG chatbot, auth, personalization, and Urdu translation features.

All subsequent `/sp.specify` prompts will decompose each module into:
- Chapters (e.g., “ROS 2 Nodes & Topics”)
- Lessons (e.g., “Writing Your First Publisher in rclpy”)
- Labs (e.g., “Simulate a URDF Humanoid with IMU in Gazebo”)

----------------------------------------------------------------

/sp.plan

Create a technical execution plan for **Project 1**: the AI-native textbook “Physical AI & Humanoid Robotics.”

---

Objectives:

- Map the 4-module curriculum to a Docusaurus-ready content architecture
- Define the AI-assisted writing workflow using Spec-Kit Plus and Claude Code
- Ensure all labs are reproducible in simulation environments (Gazebo/Isaac Sim)
- Guarantee alignment with Panaversity’s AI-native textbook standards
- Prepare foundation for future integration with Project 2 (RAG chatbot, auth, etc.)

---

Deliverables:

1. **Content Pipeline Architecture**  
   - Workflow: `/sp.specify` → Claude Code draft → human review → Markdown → Docusaurus build → GitHub Pages or Vercel  
   - AI tooling: Spec-Kit Plus for spec validation, Claude Code for drafting under guardrails  
   - Versioning: Git branches per module, tags per release  
   - Asset management: code/ (runnable scripts), assets/ (diagrams, URDFs), labs/ (step-by-step guides)

2. **Book Structure (Docusaurus-Compatible)**  
   - Sidebar: Modules → Chapters → Lessons  
   - Metadata: SEO title, description, keywords in frontmatter  
   - Cross-linking: “See also”, “Prerequisites”, “Next Lab”  
   - Responsive design for code snippets and simulation screenshots

3. **Research & Verification Strategy**  
   - Concurrent research: verify ROS 2 APIs against docs.ros.org, Isaac Sim against developer.nvidia.com  
   - Trusted sources only: no Stack Overflow heuristics without official backup  
   - APA-style inline citations for algorithms, architectures, or non-obvious claims

4. **Quality & Validation Protocol**  
   - Code testing: all Python/ROS 2 snippets validated in Dockerized Ubuntu 22.04 + ROS 2 Humble  
   - Build validation: `npm run build` must succeed with zero warnings  
   - Link checker: `lychee` or `markdown-link-check` in CI  
   - Lab reproducibility: each lab includes environment setup + expected output

5. **Key Technical Decisions (Documented with Rationale)**  
   - ROS 2 version: Humble (LTS) for stability  
   - Simulation: Gazebo Harmonic + Isaac Sim (Omniverse)  
   - Language: Python (rclpy) for accessibility  
   - OS: Ubuntu 22.04 (official ROS 2 support)  
   - Hardware guidance: clear tiers (on-prem RTX vs cloud g5.2xlarge) with cost/performance tradeoffs

---

Technical Execution Phases:

- **Phase 1 (Foundation)**: Initialize Docusaurus repo, define sidebar, set up CI linting  
- **Phase 2 (Spec Expansion)**: Run `/sp.specify` per module → chapter → lesson  
- **Phase 3 (AI Drafting)**: Use Claude Code under Spec-Kit Plus guardrails  
- **Phase 4 (Validation)**: Test code, verify links, human-edit for clarity  
- **Phase 5 (Publish)**: Deploy to GitHub Pages or Vercel validate live site

---

Success Criteria:

- Plan enables immediate `/sp.specify` of Module 1 Chapter 1  
- All labs are simulation-only (no physical robot required)  
- Book is fully navigable, searchable, and self-contained  
- Repository is forkable and reusable by other educators  
- Meets all Project 1 requirements in hackathon brief (no bonus features yet)