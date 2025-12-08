/**
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module1-ros2/README',
        'module1-ros2/chapter1-middleware',
        'module1-ros2/chapter2-nodes',
        'module1-ros2/chapter3-topics',
        'module1-ros2/chapter4-services',
        'module1-ros2/quiz',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo & Digital Twin',
      items: [
        'module2-digital-twin/README',
        'module2-digital-twin/chapter1-introduction',
        'module2-digital-twin/chapter2-world-creation',
        'module2-digital-twin/chapter3-robot-modeling',
        'module2-digital-twin/chapter4-sensors-perception',
        'module2-digital-twin/quiz',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac & Isaac ROS',
      items: [
        'module3-nvidia-isaac/README',
        'module3-nvidia-isaac/chapter1-introduction',
        'module3-nvidia-isaac/chapter2-isaac-sim',
        'module3-nvidia-isaac/chapter3-isaac-ros',
        'module3-nvidia-isaac/chapter4-jetson-deployment',
        'module3-nvidia-isaac/quiz',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4-vla/README',
        'module4-vla/chapter1-introduction',
        'module4-vla/chapter2-multimodal-perception',
        'module4-vla/chapter3-nlp-control',
        'module4-vla/chapter4-action-planning',
        'module4-vla/quiz',
      ],
      collapsed: false,
    },
  ],
};

module.exports = sidebars;
