# Data Model: High-Level Book Architecture – Physical AI & Humanoid Robotics

## Entity: Textbook Chapter
- **Fields**:
  - id: string (unique identifier)
  - title: string (chapter title)
  - module: string (parent module reference)
  - sequence: integer (order within module)
  - content: markdown (chapter content)
  - learning_objectives: array of strings (learning goals)
  - prerequisites: array of strings (required knowledge)
  - estimated_duration: integer (minutes to complete)
  - created_at: datetime
  - updated_at: datetime
  - status: enum (draft, review, published, archived)

- **Relationships**:
  - Belongs to one Module
  - Has many Lessons (composition)
  - Has many LabExercises (composition)

## Entity: Lesson
- **Fields**:
  - id: string (unique identifier)
  - title: string (lesson title)
  - chapter_id: string (parent chapter reference)
  - sequence: integer (order within chapter)
  - content: markdown (lesson content)
  - type: enum (concept, visual, code, lab, reflection)
  - estimated_duration: integer (minutes to complete)
  - created_at: datetime
  - updated_at: datetime
  - status: enum (draft, review, published, archived)

- **Relationships**:
  - Belongs to one Chapter
  - Belongs to one Module (through chapter)

## Entity: LabExercise
- **Fields**:
  - id: string (unique identifier)
  - title: string (exercise title)
  - chapter_id: string (parent chapter reference)
  - sequence: integer (order within chapter)
  - objectives: array of strings (what learner will accomplish)
  - prerequisites: array of strings (requirements to start)
  - steps: array of objects (detailed instructions)
  - expected_output: string (what learner should see)
  - assets: array of strings (required files/resources)
  - estimated_duration: integer (minutes to complete)
  - created_at: datetime
  - updated_at: datetime
  - status: enum (draft, review, published, archived)

- **Relationships**:
  - Belongs to one Chapter
  - Belongs to one Module (through chapter)

## Entity: CodeExample
- **Fields**:
  - id: string (unique identifier)
  - title: string (example title)
  - language: string (python, cpp, etc.)
  - code: string (the actual code)
  - description: string (what the code does)
  - chapter_id: string (associated chapter)
  - lesson_id: string (associated lesson, optional)
  - dependencies: array of strings (required packages/libraries)
  - expected_output: string (what code should produce)
  - created_at: datetime
  - updated_at: datetime
  - status: enum (draft, review, validated, archived)

- **Relationships**:
  - Belongs to one Chapter
  - Optionally belongs to one Lesson
  - Belongs to one Module (through chapter)

## Entity: LearnerProgress
- **Fields**:
  - id: string (unique identifier)
  - user_id: string (learner identifier)
  - chapter_id: string (chapter being tracked)
  - lesson_id: string (optional, specific lesson progress)
  - completion_percentage: float (0-100)
  - started_at: datetime
  - completed_at: datetime (null if not completed)
  - time_spent: integer (seconds spent on content)
  - quiz_scores: array of objects (if applicable)
  - lab_status: enum (not_started, in_progress, completed, verified)

- **Relationships**:
  - Belongs to one User (learner)
  - Belongs to one Chapter
  - Optionally belongs to one Lesson

## Entity: Module
- **Fields**:
  - id: string (unique identifier)
  - title: string (module title)
  - sequence: integer (order in curriculum)
  - description: string (module overview)
  - learning_outcomes: array of strings (what learner will achieve)
  - prerequisites: array of strings (required knowledge)
  - estimated_duration: integer (total minutes for module)
  - pedagogical_phase: enum (spec, sim, code, ai, integrate, deploy)
  - created_at: datetime
  - updated_at: datetime
  - status: enum (draft, review, published, archived)

- **Relationships**:
  - Has many Chapters (composition)
  - Has many Lessons (through chapters)
  - Has many LabExercises (through chapters)

## Entity: LocalizationContent
- **Fields**:
  - id: string (unique identifier)
  - original_content_id: string (reference to English content)
  - language_code: string (e.g., 'en', 'ur')
  - content: string or markdown (localized content)
  - translator_notes: string (context for translators)
  - reviewed_by: string (reviewer identifier)
  - reviewed_at: datetime
  - status: enum (translated, reviewed, published)

- **Relationships**:
  - References original content (Chapter, Lesson, LabExercise, etc.)