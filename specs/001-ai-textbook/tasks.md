---
description: "Task list for AI-native textbook project"
---

# Tasks: High-Level Book Architecture – Physical AI & Humanoid Robotics

**Input**: Design documents from `/specs/001-ai-textbook/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend (Docusaurus)**: `docs/` (content), `src/` (frontend components), `static/` (assets)
- **Backend**: `api/` (backend services), `services/` (business logic)
- **Code Examples**: `code/` (ROS 2 examples, launch files, URDF models)
- Paths shown below assume proper frontend/backend separation with Docusaurus frontend and dedicated API services

<!--
  ============================================================================
  IMPORTANT: The tasks below are SAMPLE TASKS for illustration purposes only.

  The /sp.tasks command MUST replace these with actual tasks based on:
  - User stories from spec.md (with their priorities P1, P2, P3...)
  - Feature requirements from plan.md
  - Entities from data-model.md
  - Endpoints from contracts/

  Tasks MUST be organized by user story so each story can be:
  - Implemented independently
  - Tested independently
  - Delivered as an MVP increment

  DO NOT keep these sample tasks in the generated tasks.md file.
  ============================================================================
-->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create project structure per implementation plan with docs/, src/, static/, code/, i18n/ directories
- [ ] T002 [P] Initialize Docusaurus project with npm and create package.json dependencies for Docusaurus v3+
- [ ] T003 [P] Configure basic docusaurus.config.js with site metadata and basic navigation
- [ ] T004 [P] Create initial sidebars.js structure for 4 modules
- [ ] T005 [P] Create .gitignore with standard Node.js, Python, and Ubuntu/ROS 2 exclusions

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [ ] T006 [P] Set up basic content pipeline architecture following `/sp.specify` → AI draft → human edit → test → deploy workflow
- [ ] T007 [P] Create basic markdown content structure for modules, chapters, and lessons in docs/
- [ ] T008 [P] Create initial README.md with project overview and setup instructions
- [ ] T009 Create basic CI workflow in .github/workflows/ci.yml for build validation
- [ ] T010 [P] Set up backend API structure in api/ directory with basic server configuration
- [ ] T011 [P] Create services/ directory structure for business logic components

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Create First Autonomous Robot Simulation (Priority: P1) 🎯 MVP

**Goal**: Self-learners complete their first end-to-end robot simulation following the textbook's step-by-step approach.

**Independent Test**: User can successfully set up a simulated humanoid robot in Gazebo, command it to move to specific locations, and verify its actions through the simulation environment.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T012 [P] [US1] Create basic simulation test in tests/simulation/test_basic_robot.py to verify robot responds to commands in Gazebo

### Implementation for User Story 1

- [ ] T013 [P] [US1] Create Module 1 overview content in docs/module-1/_category_.json
- [ ] T014 [P] [US1] Create Chapter 1: "Introduction to Robot Simulation" in docs/module-1/chapter-1/_category_.json
- [ ] T015 [US1] Create Lesson 1: "Setting up Your First Robot" in docs/module-1/chapter-1/lesson-1.md
- [ ] T016 [US1] Create Lab Exercise: "Simulate a URDF Humanoid with IMU" in docs/module-1/chapter-1/lab-exercise.md
- [ ] T017 [P] [US1] Create basic URDF robot model in code/urdf-models/basic_humanoid.urdf
- [ ] T018 [P] [US1] Create ROS 2 launch file for basic robot simulation in code/launch-files/basic_simulation.launch.py
- [ ] T019 [US1] Create Python code example for basic robot movement in code/ros2-examples/python/basic_robot_control.py
- [ ] T020 [US1] Add Gazebo simulation configuration in code/config/gazebo_basic.yaml
- [ ] T021 [US1] Update docusaurus.config.js to include simulation viewer component for this chapter

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Understand and Implement ROS 2 Communication (Priority: P2)

**Goal**: Self-learners understand ROS 2 fundamentals and implement communication between robot components.

**Independent Test**: User can create custom ROS 2 nodes that communicate with each other, publish sensor data, and control robot components.

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T022 [P] [US2] Create ROS 2 communication test in tests/ros2/test_communication.py to verify publishers/subscribers work

### Implementation for User Story 2

- [ ] T023 [P] [US2] Create Chapter 2: "ROS 2 Communication Patterns" in docs/module-1/chapter-2/_category_.json
- [ ] T024 [US2] Create Lesson 1: "ROS 2 Nodes and Topics" in docs/module-1/chapter-2/lesson-1.md
- [ ] T025 [US2] Create Lesson 2: "Services and Actions" in docs/module-1/chapter-2/lesson-2.md
- [ ] T026 [US2] Create Lab Exercise: "Create Publishers and Subscribers" in docs/module-1/chapter-2/lab-exercise.md
- [ ] T027 [P] [US2] Create ROS 2 publisher example in code/ros2-examples/python/publisher_example.py
- [ ] T028 [P] [US2] Create ROS 2 subscriber example in code/ros2-examples/python/subscriber_example.py
- [ ] T029 [US2] Create launch file for communication demo in code/launch-files/communication_demo.launch.py
- [ ] T030 [US2] Add sensor simulation configuration in code/config/sensor_simulation.yaml
- [ ] T031 [US2] Update sidebar to include Chapter 2 content

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Integrate AI/VLA Pipelines (Priority: P3)

**Goal**: Self-learners implement Vision-Language-Action (VLA) capabilities for their robot, enabling it to interpret commands and perform complex tasks.

**Independent Test**: User can command their simulated robot using natural language and the robot correctly interprets and executes complex tasks using VLA pipelines.

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T032 [P] [US3] Create VLA pipeline test in tests/vla/test_vla_pipeline.py to verify natural language command interpretation

### Implementation for User Story 3

- [ ] T033 [P] [US3] Create Chapter 3: "Vision-Language-Action Pipelines" in docs/module-3/chapter-1/_category_.json
- [ ] T034 [US3] Create Lesson 1: "Introduction to VLA Concepts" in docs/module-3/chapter-1/lesson-1.md
- [ ] T035 [US3] Create Lab Exercise: "Implement Natural Language Robot Control" in docs/module-3/chapter-1/lab-exercise.md
- [ ] T036 [P] [US3] Create VLA pipeline example in code/ros2-examples/python/vla_pipeline.py
- [ ] T037 [US3] Create computer vision node in code/ros2-examples/python/vision_node.py
- [ ] T038 [US3] Create natural language processing module in code/ros2-examples/python/nlp_module.py
- [ ] T039 [US3] Create launch file for VLA demo in code/launch-files/vla_demo.launch.py
- [ ] T040 [US3] Add VLA configuration in code/config/vla_config.yaml
- [ ] T041 [US3] Update sidebar to include Chapter 3 content

**Checkpoint**: All user stories should now be independently functional

---

[Add more user story phases as needed, following the same pattern]

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T042 [P] Create localization files for Urdu in i18n/ur/ following data-model.md LocalizationContent entity
- [ ] T043 [P] Add code examples validation scripts in scripts/validate-code-examples.sh
- [ ] T044 [P] Update docusaurus.config.js with SEO optimization features
- [ ] T045 [P] Create link validation workflow using lychee in .github/workflows/link-check.yml
- [ ] T046 Run quickstart.md validation to ensure all setup instructions work correctly

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Content before code examples
- Code examples before launch files
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Content creation within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all content creation for User Story 1 together:
T013 Create Module 1 overview content in docs/module-1/_category_.json
T014 Create Chapter 1: "Introduction to Robot Simulation" in docs/module-1/chapter-1/_category_.json

# Launch all code examples for User Story 1 together:
T017 Create basic URDF robot model in code/urdf-models/basic_humanoid.urdf
T018 Create ROS 2 launch file for basic robot simulation in code/launch-files/basic_simulation.launch.py
T019 Create Python code example for basic robot movement in code/ros2-examples/python/basic_robot_control.py
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence