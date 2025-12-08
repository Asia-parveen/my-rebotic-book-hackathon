import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

function ModuleCard({ title, summary, link, chapters = [] }) {
  return (
    <div style={{
      border: '2px solid #cbd5e1',
      borderRadius: 16,
      padding: 24,
      minWidth: 260,
      maxWidth: 340,
      background: 'linear-gradient(145deg, #ffffff, #f0fdf4)',
      boxShadow: '0 10px 25px rgba(0,0,0,0.08), inset 0 1px 0 rgba(255,255,255,0.8)',
      transition: 'all 0.3s ease',
      position: 'relative',
      overflow: 'hidden',
      transform: 'translateY(0)',
    }}
    onMouseEnter={(e) => {
      e.target.style.transform = 'translateY(-8px)';
      e.target.style.boxShadow = '0 20px 40px rgba(0,0,0,0.15), inset 0 1px 0 rgba(255,255,255,0.8)';
    }}
    onMouseLeave={(e) => {
      e.target.style.transform = 'translateY(0)';
      e.target.style.boxShadow = '0 10px 25px rgba(0,0,0,0.08), inset 0 1px 0 rgba(255,255,255,0.8)';
    }}
    >
      {/* Decorative element */}
      <div style={{
        position: 'absolute',
        top: 0,
        right: 0,
        width: 60,
        height: 60,
        background: 'linear-gradient(45deg, #10b981, #059669)',
        clipPath: 'polygon(0 0, 100% 0, 100% 100%)',
      }}></div>

      <h3 style={{
        marginTop: 0,
        marginBottom: 12,
        fontSize: '1.3rem',
        color: '#1e293b',
        position: 'relative',
        zIndex: 1
      }}>
        {title}
      </h3>

      <p style={{
        marginBottom: 16,
        color: '#64748b',
        lineHeight: 1.5,
        position: 'relative',
        zIndex: 1
      }}>
        {summary}
      </p>

      <div style={{
        marginBottom: 16,
        position: 'relative',
        zIndex: 1
      }}>
        <h4 style={{
          margin: '12px 0 8px',
          fontSize: '0.95rem',
          color: '#374151',
          fontWeight: '600'
        }}>
          Chapters:
        </h4>
        <ul style={{
          margin: '0 0 12px 16px',
          padding: 0,
          textAlign: 'left',
          listStylePosition: 'inside'
        }}>
          {chapters.map((chapter, index) => (
            <li key={index} style={{
              marginBottom: '6px',
              fontSize: '0.85rem',
              lineHeight: 1.4
            }}>
              <Link
                to={chapter.link}
                style={{
                  fontSize: '0.85rem',
                  color: '#059669',
                  textDecoration: 'none',
                  fontWeight: 500,
                  transition: 'color 0.2s ease'
                }}
                onMouseEnter={(e) => e.target.style.color = '#047857'}
                onMouseLeave={(e) => e.target.style.color = '#059669'}
              >
                {chapter.title}
              </Link>
            </li>
          ))}
        </ul>
      </div>
    </div>
  );
}

export default function Home() {
  const cards = [
    {
      title: 'Module 1 — ROS 2 Fundamentals',
      summary: 'Middleware (DDS), Nodes, Topics, Services — core concepts and examples.',
      link: '/docs/module1-ros2/README',
      chapters: [
        { title: 'Chapter 1: ROS 2 Middleware', link: '/docs/module1-ros2/chapter1-middleware' },
        { title: 'Chapter 2: ROS 2 Nodes', link: '/docs/module1-ros2/chapter2-nodes' },
        { title: 'Chapter 3: ROS 2 Topics', link: '/docs/module1-ros2/chapter3-topics' },
        { title: 'Chapter 4: ROS 2 Services', link: '/docs/module1-ros2/chapter4-services' },
        { title: 'Quiz', link: '/docs/module1-ros2/quiz' }
      ]
    },
    {
      title: 'Module 2 — Gazebo & Digital Twin',
      summary: 'Introduction to simulation and digital twin pipelines (Gazebo / Unity).',
      link: '/docs/module2-digital-twin/README',
      chapters: [
        { title: 'Chapter 1: Introduction to Digital Twins', link: '/docs/module2-digital-twin/chapter1-introduction' },
        { title: 'Chapter 2: Gazebo World Creation', link: '/docs/module2-digital-twin/chapter2-world-creation' },
        { title: 'Chapter 3: Robot Modeling for Simulation', link: '/docs/module2-digital-twin/chapter3-robot-modeling' },
        { title: 'Chapter 4: Sensor Integration', link: '/docs/module2-digital-twin/chapter4-sensors-perception' },
        { title: 'Quiz', link: '/docs/module2-digital-twin/quiz' }
      ]
    },
    {
      title: 'Module 3 — NVIDIA Isaac & Isaac ROS',
      summary: 'Simulation + Isaac ROS integrations and examples.',
      link: '/docs/module3-nvidia-isaac/README',
      chapters: [
        { title: 'Chapter 1: Introduction to NVIDIA Isaac', link: '/docs/module3-nvidia-isaac/chapter1-introduction' },
        { title: 'Chapter 2: Isaac Sim Fundamentals', link: '/docs/module3-nvidia-isaac/chapter2-isaac-sim' },
        { title: 'Chapter 3: Isaac ROS Packages', link: '/docs/module3-nvidia-isaac/chapter3-isaac-ros' },
        { title: 'Chapter 4: Jetson Deployment', link: '/docs/module3-nvidia-isaac/chapter4-jetson-deployment' },
        { title: 'Quiz', link: '/docs/module3-nvidia-isaac/quiz' }
      ]
    },
    {
      title: 'Module 4 — Vision-Language-Action (VLA)',
      summary: 'Vision and language integration for robot behaviour and action planning.',
      link: '/docs/module4-vla/README',
      chapters: [
        { title: 'Chapter 1: Introduction to VLA Systems', link: '/docs/module4-vla/chapter1-introduction' },
        { title: 'Chapter 2: Multimodal Perception', link: '/docs/module4-vla/chapter2-multimodal-perception' },
        { title: 'Chapter 3: NLP for Robot Control', link: '/docs/module4-vla/chapter3-nlp-control' },
        { title: 'Chapter 4: Action Planning', link: '/docs/module4-vla/chapter4-action-planning' },
        { title: 'Quiz', link: '/docs/module4-vla/quiz' }
      ]
    }
  ];

  return (
    <Layout title="Physical AI & Humanoid Robotics Book" description="A textbook for Physical AI and Humanoid Robotics.">
      <main style={{
        padding: '48px 20px',
        background: '#f6f7fb',
        minHeight: '80vh',
        position: 'relative',
        overflow: 'hidden'
      }}>
        <section style={{
          maxWidth: 1200,
          margin: '0 auto',
          textAlign: 'center',
          position: 'relative',
          zIndex: 1
        }}>
          <h1 style={{
            margin: '0 0 16px',
            fontSize: '3.2rem',
            fontWeight: '800',
            color: 'white',
            textShadow: '0 4px 8px rgba(0,0,0,0.4), 0 2px 4px rgba(0,0,0,0.3)',
            animation: 'fadeInUp 1s ease',
            padding: '25px 45px',
            background: 'linear-gradient(135deg, #10b981, #059669, #047857)',
            borderRadius: '20px',
            backdropFilter: 'blur(12px)',
            WebkitBackdropFilter: 'blur(12px)',
            border: '1px solid rgba(255, 255, 255, 0.3)',
            boxShadow: '0 10px 30px rgba(0,0,0, 0.2), inset 0 2px 4px rgba(255,255,255, 0.2)'
          }}>
            Physical AI & Humanoid Robotics Book
          </h1>
          <p style={{
            margin: '0 0 36px',
            fontSize: '1.10rem',
            color: '#374151',
            maxWidth: '700px',
            margin: '0 auto 36px',
            lineHeight: 1.6,
            textShadow: '0 1px 2px rgba(255,255,255,0.5)'
          }}>
            A comprehensive textbook for learning ROS 2, simulation, perception, and robot control.
          </p>

          <div style={{
            display: 'flex',
            gap: 24,
            justifyContent: 'center',
            flexWrap: 'wrap',
            marginTop: 24,
            animation: 'fadeIn 1.5s ease'
          }}>
            {cards.map((card, index) => (
              <div
                key={card.title}
                style={{
                  animation: `fadeInUp 0.6s ease forwards`,
                  animationDelay: `${index * 0.1}s`,
                  opacity: 0,
                  transform: 'translateY(20px)'
                }}
              >
                <ModuleCard
                  title={card.title}
                  summary={card.summary}
                  link={card.link}
                  chapters={card.chapters || []}
                />
              </div>
            ))}
          </div>

          <div style={{
            marginTop: 48,
            animation: 'fadeIn 1s ease 0.5s both',
            display: 'flex',
            gap: 16,
            justifyContent: 'center',
            flexWrap: 'wrap'
          }}>
            <div style={{
              animation: 'pulse 2s infinite 0.7s',
              animationTimingFunction: 'cubic-bezier(0.4, 0, 0.6, 1)'
            }}>
            <Link to="/docs/module1-ros2/" style={{
              padding: '12px 20px',
              borderRadius: 10,
              textDecoration: 'none',
              background: 'linear-gradient(135deg, #10b981, #059669)',
              color: 'white',
              fontWeight: '600',
              fontSize: '0.9rem',
              border: 'none',
              transition: 'all 0.4s cubic-bezier(0.25, 0.8, 0.25, 1)',
              boxShadow: '0 6px 16px rgba(16, 185, 129, 0.4)',
              display: 'inline-flex',
              alignItems: 'center',
              gap: '6px',
              position: 'relative',
              overflow: 'hidden',
              transform: 'translateY(0)'
            }}
            onMouseEnter={(e) => {
              e.target.style.transform = 'translateY(-5px) scale(1.05)';
              e.target.style.boxShadow = '0 12px 25px rgba(16, 185, 129, 0.6)';
              // Add animation effect
              e.target.style.background = 'linear-gradient(135deg, #059669, #10b981)';
              // Animate the shimmer effect
              const shimmer = e.target.querySelector('span:nth-child(2)');
              if (shimmer) {
                shimmer.style.left = '100%';
              }
            }}
            onMouseLeave={(e) => {
              e.target.style.transform = 'translateY(0) scale(1)';
              e.target.style.boxShadow = '0 6px 16px rgba(16, 185, 129, 0.4)';
              e.target.style.background = 'linear-gradient(135deg, #10b981, #059669)';
              // Reset the shimmer effect
              const shimmer = e.target.querySelector('span:nth-child(2)');
              if (shimmer) {
                shimmer.style.left = '-100px';
              }
            }}>
              <span style={{position: 'relative', zIndex: 1}}>Module 1</span>
              <span style={{
                position: 'absolute',
                top: 0,
                left: -100,
                width: '100px',
                height: '100%',
                background: 'rgba(255, 255, 255, 0.3)',
                transform: 'skewX(-25deg)',
                transition: 'left 0.5s ease',
                zIndex: 0
              }}></span>
            </Link>
            </div>
            <div style={{
              animation: 'pulse 2s infinite 0.9s',
              animationTimingFunction: 'cubic-bezier(0.4, 0, 0.6, 1)'
            }}>
            <Link to="/docs/module2-digital-twin/" style={{
              padding: '12px 20px',
              borderRadius: 10,
              textDecoration: 'none',
              background: 'linear-gradient(135deg, #059669, #047857)',
              color: 'white',
              fontWeight: '600',
              fontSize: '0.9rem',
              border: 'none',
              transition: 'all 0.4s cubic-bezier(0.25, 0.8, 0.25, 1)',
              boxShadow: '0 6px 16px rgba(5, 150, 105, 0.4)',
              display: 'inline-flex',
              alignItems: 'center',
              gap: '6px',
              position: 'relative',
              overflow: 'hidden',
              transform: 'translateY(0)'
            }}
            onMouseEnter={(e) => {
              e.target.style.transform = 'translateY(-5px) scale(1.05)';
              e.target.style.boxShadow = '0 12px 25px rgba(5, 150, 105, 0.6)';
              // Add animation effect
              e.target.style.background = 'linear-gradient(135deg, #047857, #059669)';
              // Animate the shimmer effect
              const shimmer = e.target.querySelector('span:nth-child(2)');
              if (shimmer) {
                shimmer.style.left = '100%';
              }
            }}
            onMouseLeave={(e) => {
              e.target.style.transform = 'translateY(0) scale(1)';
              e.target.style.boxShadow = '0 6px 16px rgba(5, 150, 105, 0.4)';
              e.target.style.background = 'linear-gradient(135deg, #059669, #047857)';
              // Reset the shimmer effect
              const shimmer = e.target.querySelector('span:nth-child(2)');
              if (shimmer) {
                shimmer.style.left = '-100px';
              }
            }}>
              <span style={{position: 'relative', zIndex: 1}}>Module 2</span>
              <span style={{
                position: 'absolute',
                top: 0,
                left: -100,
                width: '100px',
                height: '100%',
                background: 'rgba(255, 255, 255, 0.3)',
                transform: 'skewX(-25deg)',
                transition: 'left 0.5s ease',
                zIndex: 0
              }}></span>
            </Link>
            </div>
            <div style={{
              animation: 'pulse 2s infinite 1.1s',
              animationTimingFunction: 'cubic-bezier(0.4, 0, 0.6, 1)'
            }}>
            <Link to="/docs/module3-nvidia-isaac/" style={{
              padding: '12px 20px',
              borderRadius: 10,
              textDecoration: 'none',
              background: 'linear-gradient(135deg, #047857, #065f46)',
              color: 'white',
              fontWeight: '600',
              fontSize: '0.9rem',
              border: 'none',
              transition: 'all 0.4s cubic-bezier(0.25, 0.8, 0.25, 1)',
              boxShadow: '0 6px 16px rgba(4, 120, 87, 0.4)',
              display: 'inline-flex',
              alignItems: 'center',
              gap: '6px',
              position: 'relative',
              overflow: 'hidden',
              transform: 'translateY(0)'
            }}
            onMouseEnter={(e) => {
              e.target.style.transform = 'translateY(-5px) scale(1.05)';
              e.target.style.boxShadow = '0 12px 25px rgba(4, 120, 87, 0.6)';
              // Add animation effect
              e.target.style.background = 'linear-gradient(135deg, #065f46, #047857)';
              // Animate the shimmer effect
              const shimmer = e.target.querySelector('span:nth-child(2)');
              if (shimmer) {
                shimmer.style.left = '100%';
              }
            }}
            onMouseLeave={(e) => {
              e.target.style.transform = 'translateY(0) scale(1)';
              e.target.style.boxShadow = '0 6px 16px rgba(4, 120, 87, 0.4)';
              e.target.style.background = 'linear-gradient(135deg, #047857, #065f46)';
              // Reset the shimmer effect
              const shimmer = e.target.querySelector('span:nth-child(2)');
              if (shimmer) {
                shimmer.style.left = '-100px';
              }
            }}>
              <span style={{position: 'relative', zIndex: 1}}>Module 3</span>
              <span style={{
                position: 'absolute',
                top: 0,
                left: -100,
                width: '100px',
                height: '100%',
                background: 'rgba(255, 255, 255, 0.3)',
                transform: 'skewX(-25deg)',
                transition: 'left 0.5s ease',
                zIndex: 0
              }}></span>
            </Link>
            <Link to="/docs/module4-vla/" style={{
              padding: '12px 20px',
              borderRadius: 10,
              textDecoration: 'none',
              background: 'linear-gradient(135deg, #065f46, #064e3b)',
              color: 'white',
              fontWeight: '600',
              fontSize: '0.9rem',
              border: 'none',
              transition: 'all 0.4s cubic-bezier(0.25, 0.8, 0.25, 1)',
              boxShadow: '0 6px 16px rgba(6, 95, 70, 0.4)',
              display: 'inline-flex',
              alignItems: 'center',
              gap: '6px',
              position: 'relative',
              overflow: 'hidden',
              transform: 'translateY(0)'
            }}
            onMouseEnter={(e) => {
              e.target.style.transform = 'translateY(-5px) scale(1.05)';
              e.target.style.boxShadow = '0 12px 25px rgba(6, 95, 70, 0.6)';
              // Add animation effect
              e.target.style.background = 'linear-gradient(135deg, #064e3b, #065f46)';
              // Animate the shimmer effect
              const shimmer = e.target.querySelector('span:nth-child(2)');
              if (shimmer) {
                shimmer.style.left = '100%';
              }
            }}
            onMouseLeave={(e) => {
              e.target.style.transform = 'translateY(0) scale(1)';
              e.target.style.boxShadow = '0 6px 16px rgba(6, 95, 70, 0.4)';
              e.target.style.background = 'linear-gradient(135deg, #065f46, #064e3b)';
              // Reset the shimmer effect
              const shimmer = e.target.querySelector('span:nth-child(2)');
              if (shimmer) {
                shimmer.style.left = '-100px';
              }
            }}>
              <span style={{position: 'relative', zIndex: 1}}>Module 4</span>
              <span style={{
                position: 'absolute',
                top: 0,
                left: -100,
                width: '100px',
                height: '100%',
                background: 'rgba(255, 255, 255, 0.3)',
                transform: 'skewX(-25deg)',
                transition: 'left 0.5s ease',
                zIndex: 0
              }}></span>
            </Link>
            </div>
          </div>
        </section>

        <style>{`
          @keyframes fadeInUp {
            from {
              opacity: 0;
              transform: translateY(20px);
            }
            to {
              opacity: 1;
              transform: translateY(0);
            }
          }

          @keyframes fadeIn {
            from {
              opacity: 0;
            }
            to {
              opacity: 1;
            }
          }

          @keyframes pulse {
            0%, 100% {
              transform: scale(1);
              opacity: 1;
            }
            50% {
              transform: scale(1.03);
              opacity: 0.95;
            }
          }
        `}</style>
      </main>
    </Layout>
  );
}
