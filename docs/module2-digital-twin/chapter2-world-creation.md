---
sidebar_position: 3
title: Chapter 2 - Gazebo World Creation
---

# Chapter 2: Gazebo Fundamentals and World Creation

## Understanding Gazebo's Physics Engine and Rendering Pipeline

Gazebo's architecture is built around a modular design that separates the physics engine, rendering engine, and user interface. The physics engine handles the simulation of rigid body dynamics, collisions, and joint constraints, while the rendering engine provides visual feedback of the simulation.

### Physics Engine Components

The physics engine in Gazebo manages:

*   **Collision Detection**: Identifying when objects intersect or come into contact
*   **Dynamics Simulation**: Calculating forces, torques, and resulting motions
*   **Joint Constraints**: Managing the degrees of freedom between connected bodies
*   **Contact Processing**: Handling friction, restitution, and contact forces

### Rendering Pipeline

The rendering pipeline provides visual representation of the simulation:

*   **Scene Graph**: Hierarchical representation of all objects in the world
*   **Lighting System**: Directional, point, and spot lights for realistic illumination
*   **Materials and Textures**: Surface properties for visual realism
*   **Camera Views**: Multiple viewpoints for visualization and sensor simulation

## Creating Basic Simulation Worlds and Environments

Gazebo worlds are defined using the Simulation Description Format (SDF), an XML-based format that describes the entire simulation environment. A basic world file includes:

*   **World Properties**: Gravity, magnetic field, and atmospheric settings
*   **Models**: Robot and object definitions within the environment
*   **Lights**: Lighting configuration for the scene
*   **Physics Engine**: Configuration of the underlying physics engine

### Basic World Structure

A minimal Gazebo world file includes:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- World properties -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Models and objects -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.3 -0.9</direction>
    </light>
  </world>
</sdf>
```

## Working with Gazebo's Coordinate Systems and Units

Gazebo uses a right-handed coordinate system where:

*   **X-axis**: Points forward (in the default orientation)
*   **Y-axis**: Points to the left (in the default orientation)
*   **Z-axis**: Points upward (opposite to gravity direction)

All measurements in Gazebo use the International System of Units (SI):

*   **Distance**: Meters (m)
*   **Mass**: Kilograms (kg)
*   **Time**: Seconds (s)
*   **Velocity**: Meters per second (m/s)
*   **Acceleration**: Meters per second squared (m/s²)

Understanding these conventions is crucial for correctly placing objects, defining robot models, and interpreting sensor data.

## Adding Static and Dynamic Objects to the Simulation

Objects in Gazebo can be either static (immovable) or dynamic (affected by physics). Static objects are useful for environments like walls, floors, and furniture, while dynamic objects can move, collide, and interact with forces.

### Static Objects

To make an object static, set the `<static>` property to `true` in its model definition:

```xml
<model name="table">
  <static>true</static>
  <pose>1 0 0 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <box>
          <size>1 0.8 0.8</size>
        </box>
      </geometry>
    </visual>
    <collision name="collision">
      <geometry>
        <box>
          <size>1 0.8 0.8</size>
        </box>
      </geometry>
    </collision>
  </link>
</model>
```

### Dynamic Objects

Dynamic objects are the default and will respond to physics simulation. They can be manipulated by forces, collisions, and joint actuators.

## Configuring Lighting and Visual Properties

Lighting in Gazebo affects both the visual appearance of the simulation and the behavior of vision-based sensors. Different types of lights serve different purposes:

*   **Directional Lights**: Simulate distant light sources like the sun
*   **Point Lights**: Emit light equally in all directions from a point
*   **Spot Lights**: Emit light in a cone shape with adjustable properties

### Material Properties

Materials define how surfaces interact with light:

*   **Diffuse Color**: The base color of the surface
*   **Specular Color**: The color of highlights and reflections
*   **Ambient Color**: The color when not directly illuminated
*   **Emission**: Color emitted by the surface itself

These properties not only affect visual appearance but also influence how simulated cameras and sensors perceive the environment.