# Physical AI & Humanoid Robotics: From Simulation to Embodied Intelligence

This repository contains an AI-native textbook for Physical AI & Humanoid Robotics. The textbook is built using [Docusaurus](https://docusaurus.io/), a modern static website generator with TypeScript support.

## Overview

This textbook enables self-learners to build and command simulated autonomous humanoid robots using ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action (VLA) pipelines. The content follows a pedagogical sequence: Spec → Sim → Code → AI → Integrate → Deploy.

## Structure

- `website/` - Docusaurus-based textbook project with TypeScript support
- `specs/` - Specification files for the textbook modules
- `docs/` - Original documentation structure (for reference)

## Local Development

First, navigate to the website directory:

```bash
cd website
```

Then install dependencies and start the development server:

```bash
npm install
npm start
```

This command starts a local development server at http://localhost:3000/ai-robo-learning/ and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

From the `website` directory:

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

The site is configured for GitHub Pages deployment. From the `website` directory:

```bash
GIT_USER=madnan-github npm run deploy
```

This command builds the website and pushes it to the `gh-pages` branch for GitHub Pages hosting.
