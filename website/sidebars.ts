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
      items: ['module-1/chapter-1/introduction'],
    },
    {
      type: 'category',
      label: 'Module 2: ROS 2 Fundamentals',
      items: ['module-2/chapter-1/ros2-fundamentals'],
    },
    {
      type: 'category',
      label: 'Module 3: Simulation with Gazebo',
      items: ['module-3/chapter-1/simulation-basics'],
    },
    {
      type: 'category',
      label: 'Module 4: AI Integration',
      items: ['module-4/chapter-1/ai-integration'],
    },
  ],
};

export default sidebars;
