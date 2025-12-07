import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import HomePageHero from '../components/HomePageHero';
import FeatureGrid from '../components/FeatureGrid';
import WeeklyTimeline from '../components/WeeklyTimeline';
import ModuleCard from '../components/ModuleCard';
import AnimatedCTA from '../components/AnimatedCTA';
import Footer from '../components/Footer';

// Define data structures
interface Module {
  title: string;
  focus: string;
  topics: string[];
  icon?: string;
  link: string;
}

interface Feature {
  title: string;
  description: string;
  icon?: string;
}

interface Week {
  range: string;
  topics: string[];
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  // Content for HomePageHero
  const heroContent = {
    title: "AI/Spec-Driven Book Creation — Physical AI & Humanoid Robotics",
    subtitle: "Bridging the gap between the digital brain and the physical body. Students apply their AI knowledge to control Humanoid Robots in simulated and real-world environments.",
    ctaText: "Start Learning",
    ctaLink: "/docs/intro", // Placeholder, will link to first chapter
  };

  // Content for ModuleCard (for homepage display)
  const modules: Module[] = [
    {
      title: "ROS 2 — Robotic Nervous System",
      focus: "Middleware for robot control.",
      topics: [
        "ROS 2 Nodes, Topics, and Services.",
        "Bridging Python Agents to ROS controllers using rclpy.",
        "Understanding URDF (Unified Robot Description Format) for humanoids.",
      ],
      link: "/docs/chapter1", // Placeholder
    },
    {
      title: "Digital Twin Simulation — Gazebo & Unity",
      focus: "Physics simulation and environment building.",
      topics: [
        "Simulating physics, gravity, and collisions in Gazebo.",
        "High-fidelity rendering and human-robot interaction in Unity.",
        "Simulating sensors: LiDAR, Depth Cameras, and IMUs.",
      ],
      link: "/docs/chapter2", // Placeholder
    },
    {
      title: "NVIDIA Isaac — AI Robot Brain",
      focus: "Advanced perception and training.",
      topics: [
        "NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation.",
        "Isaac ROS: Hardware-accelerated VSLAM (Visual SLAM) and navigation.",
        "Nav2: Path planning for bipedal humanoid movement.",
      ],
      link: "/docs/chapter3", // Placeholder
    },
    {
      title: "Vision-Language-Action Robotics (VLA)",
      focus: "The convergence of LLMs and Robotics.",
      topics: [
        "Voice-to-Action: Using OpenAI Whisper for voice commands.",
        `Cognitive Planning: Using LLMs to translate natural language ("Clean the room") into a sequence of ROS 2 actions.`,
        "Capstone Project: The Autonomous Humanoid.",
      ],
      link: "/docs/chapter4", // Placeholder
    },
  ];

  // Content for WeeklyTimeline
  const weeklyBreakdown: Week[] = [
    {
      range: "Weeks 1-2",
      topics: [
        "Foundations of Physical AI and embodied intelligence",
        "From digital AI to AI that understand physical laws",
        "Overview of humanoid robotics landscape",
        "Sensor systems: LIDAR, cameras, IMUs, force/torque sensors",
      ],
    },
    {
      range: "Weeks 3-5",
      topics: [
        "ROS 2 architecture and core concepts",
        "Nodes, topics, services, and actions",
        "Building ROS 2 packages with Python",
        "Launch files and parameter management",
      ],
    },
    {
      range: "Weeks 6-7",
      topics: [
        "Gazebo simulation environment setup",
        "URDF and SDF robot description formats",
        "Physics simulation and sensor simulation",
        "Introduction to Unity for robot visualization",
      ],
    },
    {
      range: "Weeks 8-10",
      topics: [
        "NVIDIA Isaac SDK and Isaac Sim",
        "AI-powered perception and manipulation",
        "Reinforcement learning for robot control",
        "Sim-to-real transfer techniques",
      ],
    },
    {
      range: "Weeks 11-12",
      topics: [
        "Humanoid robot kinematics and dynamics",
        "Bipedal locomotion and balance control",
        "Manipulation and grasping with humanoid hands",
        "Natural human-robot interaction design",
      ],
    },
    {
      range: "Week 13",
      topics: [
        "Integrating GPT models for conversational AI in robots",
        "Speech recognition and natural language understanding",
        "Multi-modal interaction: speech, gesture, vision",
        "Assessments",
      ],
    },
  ];

  // Placeholder content for FeatureGrid
  const features: Feature[] = [
    {
      title: "Embodied Intelligence",
      description: "Learn to build AI systems that interact with the physical world.",
      icon: "/img/robot-icon.svg", // Placeholder icon
    },
    {
      title: "ROS 2 Mastery",
      description: "Develop robust robotic applications with the Robot Operating System 2.",
      icon: "/img/ros2-icon.svg", // Placeholder icon
    },
    {
      title: "Advanced Simulation",
      description: "Utilize Gazebo and Unity for realistic digital twin environments.",
      icon: "/img/simulation-icon.svg", // Placeholder icon
    },
    {
      title: "AI-Powered Robotics",
      description: "Integrate NVIDIA Isaac for cutting-edge perception and training.",
      icon: "/img/nvidia-icon.svg", // Placeholder icon
    },
    {
      title: "VLA Systems",
      description: "Bridge vision, language, and action for intelligent robotic control.",
      icon: "/img/vla-icon.svg", // Placeholder icon
    },
    {
      title: "Humanoid Applications",
      description: "Explore the development and control of bipedal robots.",
      icon: "/img/humanoid-icon.svg", // Placeholder icon
    },
  ];

  const footerLinks = [
    { text: "Documentation", href: "/docs/intro" },
    { text: "GitHub", href: "https://github.com/your-org/your-repo" }, // Placeholder
  ];

  // Hardware Requirements Summary (concise for homepage)
  const hardwareSummary = `This course requires high-performance workstations (NVIDIA RTX GPU, Intel i7/Ryzen 9, 64GB RAM, Ubuntu 22.04 LTS) for physics simulation, visual perception, and generative AI tasks. Optional Edge AI Kits (NVIDIA Jetson) and physical robots (Unitree Go2/G1) are discussed for hands-on learning.`;

  return (
    <Layout
      title={`Home`}
      description="Learn Physical AI & Humanoid Robotics with our comprehensive Docusaurus book.">
      <main>
        <HomePageHero {...heroContent} />

        <div className="container margin-bottom--xl">
          <section className="text--center margin-bottom--xl">
            <h2 className="hero__subtitle">Quarter Overview</h2>
            <p>{heroContent.subtitle}</p>
          </section>

          <section className="margin-bottom--xl">
            <h2 className="text--center">Why Physical AI Matters</h2>
            <p className="text--justify">
              Humanoid robots are poised to excel in our human-centered world because they share our physical form and can be trained with abundant data from interacting in human environments. This represents a significant transition from AI models confined to digital environments to embodied intelligence that operates in physical space.
            </p>
          </section>

          <FeatureGrid features={features} />

          <section className="margin-top--xl margin-bottom--xl">
            <h2 className="text--center">Learning Modules</h2>
            <div className="row">
              {modules.map((module, idx) => (
                <div key={idx} className="col col--6 margin-bottom--lg">
                  <ModuleCard module={module} />
                </div>
              ))}
            </div>
          </section>

          <WeeklyTimeline weeks={weeklyBreakdown} />

          <section className="text--center margin-top--xl margin-bottom--xl">
            <h2 className="hero__subtitle">Hardware Requirements</h2>
            <p>{hardwareSummary}</p>
            <p>
              For detailed hardware requirements, please refer to the course documentation.
            </p>
          </section>

          <section className="text--center margin-top--xl margin-bottom--xl">
            <AnimatedCTA text="View Course Details" link="/docs/intro" />
          </section>
        </div>

        <Footer copyrightText={`Copyright © ${new Date().getFullYear()} ${siteConfig.title}.`} links={footerLinks} />
      </main>
    </Layout>
  );
}