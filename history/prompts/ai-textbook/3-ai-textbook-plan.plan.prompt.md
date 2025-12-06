---
id: 3
title: AI Textbook Plan
stage: plan
date: 2025-12-06
surface: agent
model: claude-opus-4-5-20251101
feature: ai-textbook
branch: 001-ai-textbook
user: ruser
command: /sp.plan
labels: ["plan", "ai-textbook", "robotics", "education", "docusaurus"]
links:
  spec: specs/001-ai-textbook/spec.md
  plan: specs/001-ai-textbook/plan.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-ai-textbook/plan.md
 - specs/001-ai-textbook/research.md
 - specs/001-ai-textbook/data-model.md
 - specs/001-ai-textbook/quickstart.md
 - specs/001-ai-textbook/contracts/progress-api.yaml
 - CLAUDE.md
tests:
 - none
---

## Prompt

```text
Create a technical execution plan for **Project 1**: the AI-native textbook "Physical AI & Humanoid Robotics."

---

Objectives:

- Map the 4-module curriculum to a Docusaurus-ready content architecture
- Define the AI-assisted writing workflow using Spec-Kit Plus and Claude Code
- Ensure all labs are reproducible in simulation environments (Gazebo/Isaac Sim)
- Guarantee alignment with Panaversity's AI-native textbook standards
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
   - Cross-linking: "See also", "Prerequisites", "Next Lab"
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
```

## Response snapshot

Created a comprehensive technical execution plan for the AI-native textbook "Physical AI & Humanoid Robotics" with all required deliverables:

1. Implementation plan document with technical context, constitution check, and project structure
2. Research document covering technology stack, content architecture, and implementation approach
3. Data model for textbook content and learner progress tracking
4. Quickstart guide for setting up the textbook environment
5. API contract for progress tracking functionality
6. Updated agent context with project-specific technologies

The plan fully aligns with the project constitution and addresses all specified objectives, with proper verification against official documentation sources and simulation environment validation.

## Outcome

- ✅ Impact: Complete technical execution plan for AI-native textbook project
- 🧪 Tests: none
- 📁 Files: 6 files created/updated (plan.md, research.md, data-model.md, quickstart.md, contracts/progress-api.yaml, CLAUDE.md)
- 🔁 Next prompts: /sp.specify for Module 1 Chapter 1 specification
- 🧠 Reflection: Plan provides solid foundation for implementing the 4-module curriculum with proper architecture and validation processes

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin Module 1 Chapter 1 specification with /sp.specify