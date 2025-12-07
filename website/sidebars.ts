import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: Physical AI & Humanoid Robotics',
      items: [
        'module-1/chapter-1/introduction',
        'module-1/chapter-2/nodes-architecture',
        'module-1/chapter-3/topics-message-passing',
        'module-1/chapter-4/services-actions',
        'module-1/chapter-5/rclpy-integration',
        'module-1/chapter-6/urdf-humanoids'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: ROS 2 Fundamentals',
      items: [
        'module-2/chapter-1/ros2-fundamentals',
        'module-2/chapter-2/gazebo-physics',
        'module-2/chapter-3/robot-models-gazebo',
        'module-2/chapter-4/sensor-simulation',
        'module-2/chapter-5/unity-integration',
        'module-2/chapter-6/environment-building'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Simulation with Gazebo',
      items: [
        'module-3/chapter-1/simulation-basics',
        'module-3/chapter-2/isaac-sim',
        'module-3/chapter-3/isaac-ros-vslam',
        'module-3/chapter-4/navigation-nav2',
        'module-3/chapter-5/path-planning-humanoids',
        'module-3/chapter-6/perception-systems'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: AI Integration',
      items: [
        'module-4/chapter-1/ai-integration',
        'module-4/chapter-2/voice-command-processing',
        'module-4/chapter-3/cognitive-planning-llms',
        'module-4/chapter-4/computer-vision-robotics',
        'module-4/chapter-5/object-manipulation',
        'module-4/chapter-6/capstone-project'
      ],
    },
  ],
};

export default sidebars;