# Autonomous Receptionist Robot

> **Status:** 🚧 Active Development

An AI-powered autonomous receptionist robot built with **ROS 2**,
**Python**, **Gemini**, and **Model Context Protocol (MCP)**. The system
is designed around a modular ROS architecture where perception, AI,
speech, and navigation are independent components that can evolve
without affecting one another.

------------------------------------------------------------------------

# Overview

The robot is designed to:

-   Detect approaching visitors using a Logitech C270 USB camera.
-   Generate intelligent greetings using Gemini through MCP.
-   Speak naturally using a dedicated Text-to-Speech pipeline.
-   Escort visitors to destinations using ROS 2 Navigation (future
    integration).
-   Fuse multiple sensors for safe autonomous navigation.
-   Provide a scalable architecture for future voice conversations and
    cloud services.

------------------------------------------------------------------------

# Hardware

  -----------------------------------------------------------------------
  Component                               Purpose
  --------------------------------------- -------------------------------
  Raspberry Pi 4                          Main onboard computer

  Logitech C270 USB Camera                Face detection, QR scanning,
                                          visual perception

  8× IR Sensors                           Close-range obstacle detection

  8× Ultrasonic Sensors                   Medium-range obstacle detection

  Omnidirectional Drive Base              Robot mobility

  Motor Controller (Embedded Team)        Low-level motor control

  Speaker                                 Voice output

  Microphone (Future)                     Speech-to-Text / Voice
                                          conversations
  -----------------------------------------------------------------------

------------------------------------------------------------------------

# Software Stack

  Layer              Technology
  ------------------ ----------------------------
  Middleware         ROS 2
  Language           Python
  Vision             OpenCV
  AI                 Gemini
  AI Communication   MCP
  Navigation         Nav2 (planned)
  SLAM               slam_toolbox (planned)
  Speech             TTS Engine (Piper planned)

------------------------------------------------------------------------

# System Architecture

``` mermaid
graph TB

subgraph Vision
CAM[Logitech C270]
CN[camera_node]
VI[vision_interaction.py]
end

subgraph Reception
RN[receptionist_node]
end

subgraph AI
LLMN[llm_node]
GI[gemini_interface.py]
MCP[MCP Server]
GEMINI[Gemini]
end

subgraph Speech
TTSN[tts_node]
TE[tts_engine.py]
SPK[Speaker]
end

subgraph Navigation
SF[sensor_fusion_node]
NAV[navigation_node]
REC[recovery_node]
MUX[cmd_vel_mux_node]
end

CAM --> CN
CN --> VI
VI -->|Person Detected| RN
RN -->|/llm_request| LLMN
LLMN --> GI
GI --> MCP
MCP --> GEMINI
GEMINI --> GI
GI --> LLMN
LLMN -->|/llm_response| RN
RN -->|/tts_request| TTSN
TTSN --> TE
TE --> SPK
SF --> NAV
NAV --> MUX
REC --> MUX
```

------------------------------------------------------------------------

# AI Greeting Pipeline

``` text
Person Appears
      ↓
Camera Node
      ↓
Face Detection
      ↓
Receptionist Node
      ↓
LLM Node
      ↓
Gemini Interface
      ↓
Gemini (via MCP)
      ↓
Receptionist Node
      ↓
TTS Node
      ↓
TTS Engine
      ↓
Speaker
```

------------------------------------------------------------------------

# Project Structure

``` text
Welcoming-Robot/
├── config/
├── core/
│   ├── astar.py
│   ├── fusion.py
│   ├── omni_controller.py
│   ├── vision_interaction.py
│   ├── vision_navigation.py
│   ├── gemini_interface.py
│   └── tts_engine.py
├── launch/
├── nodes/
│   ├── camera_node.py
│   ├── cmd_vel_mux_node.py
│   ├── llm_node.py
│   ├── navigation_node.py
│   ├── receptionist_node.py
│   ├── recovery_node.py
│   ├── sensor_fusion_node.py
│   └── tts_node.py
├── utils/
└── README.md
```

------------------------------------------------------------------------

# Component Responsibilities

  Component             Responsibility
  --------------------- ------------------------------------------
  camera_node           Camera capture and visitor detection
  receptionist_node     Workflow orchestration
  llm_node              ROS wrapper for Gemini
  gemini_interface.py   Prompt engineering and MCP communication
  tts_node              ROS wrapper for speech
  tts_engine.py         Speech synthesis implementation
  navigation_node       Navigation and path planning
  sensor_fusion_node    Multi-sensor fusion
  recovery_node         Recovery behaviors

------------------------------------------------------------------------

# Development Roadmap

### Phase 1

-   AI Greeting
-   Face Detection
-   Gemini + MCP
-   Text-to-Speech

### Phase 2

-   Speech-to-Text
-   Multi-turn Conversations
-   Visitor Intent Recognition

### Phase 3

-   Raspberry Pi Deployment
-   Hardware Integration
-   SLAM
-   Autonomous Navigation

### Phase 4

-   Escort Mode
-   Voice Conversations
-   Appointment Management
-   Cloud Integration

------------------------------------------------------------------------

# Notes

Performance metrics and CPU utilization will be documented after
real-world testing on Raspberry Pi 4. No estimated performance values
are included in this repository.

------------------------------------------------------------------------

# License

Educational and research purposes.
