<!--
Sync Impact Report:
- Version change: undefined → 1.0.0
- Modified principles: All 6 principles added
- Added sections: Key Standards, Development Workflow, Governance Rules
- Removed sections: None
- Templates requiring updates: ✅ updated
- Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics – AI-Native Textbook Constitution

## Core Principles

### Spec-driven authoring
No chapter written without a prior approved specification

### AI-augmented but human-verified content
AI-augmented but human-verified content (Claude Code + human-in-the-loop review)

### Ground truth anchoring
Ground truth anchored in official ROS 2, NVIDIA Isaac, Gazebo, and Unity documentation

### Progressive pedagogy
Progressive pedagogy: beginner → intermediate → capstone autonomy

### Open educational resource compliance
Open educational resource (OER) compliant: reusable, remixable, redistributable

### Automation-first publishing
Automation-first publishing: CI/CD-ready Docusaurus deployment to GitHub Pages or Vercel

## Key Standards
Every chapter must stem from a `/sp.specify`-approved design. All technical assertions validated against trusted sources (ROS docs, Isaac SDK, etc.). Code snippets must be executable in a standard Ubuntu 22.04 + ROS 2 Humble environment. Markdown-only content with Docusaurus frontmatter (title, description, slug, sidebar). Repository must include: Clean branch strategy (main + feature branches), Semantic commit messages, Public GitHub Issues for community feedback. Writing must follow: Concept → Visual → Code → Lab → Reflection. AI output must be edited for clarity, safety, and pedagogical effectiveness.

## Development Workflow
Content pipeline: `/sp.specify` → AI draft → human edit → test → GitHub Pages or Vercel deploy. All content must be original, non-infringing, and free of hallucinated APIs or libraries. No prose generated without explicit spec alignment. Clean branch strategy (main + feature branches), Semantic commit messages, Public GitHub Issues for community feedback.

## Constraints
Output format: Static site via Docusaurus v3+. Hosting: GitHub Pages or Vercel only (no Vercel/Netlify for base submission). Source control: Public GitHub repository. AI stack: Spec-Kit Plus + Claude Code only (no external LLM APIs in draft phase). All content must be executable in a standard Ubuntu 22.04 + ROS 2 Humble environment. No prose generated without explicit spec alignment. SEO-optimized headings, meta descriptions, and internal linking required.

## Governance
All changes must comply with the core principles. Amendments require documentation and approval. All chapters must be verified against official documentation sources. Compliance with Docusaurus best practices required. Code snippets must be tested in simulation environments (Gazebo/Isaac Sim).

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06