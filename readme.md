# Autonomous Receptionist Robot

> **Status:** 🚧 Active Development

An AI-powered autonomous receptionist robot built with **ROS 2**,
**Python**, **Gemini**, and **MCP (Model Context Protocol)**.

## Overview

-   Detect visitors using Logitech C270.
-   Generate intelligent greetings with Gemini.
-   Speak using a dedicated TTS pipeline.
-   Modular ROS2 architecture.
-   Future support for SLAM, Nav2 and voice conversations.

## Architecture

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

CAM --> CN
CN --> VI
VI --> RN
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
```

## AI Pipeline

``` text
Camera
 ↓
Face Detection
 ↓
Receptionist Node
 ↓
LLM Node
 ↓
Gemini Interface
 ↓
Gemini (MCP)
 ↓
Receptionist Node
 ↓
TTS Node
 ↓
TTS Engine
 ↓
Speaker
```

## Project Structure

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

## Responsibilities

  Component             Responsibility
  --------------------- --------------------------------------
  camera_node           Camera capture and visitor detection
  receptionist_node     Workflow orchestration
  llm_node              ROS wrapper for Gemini
  gemini_interface.py   Prompt engineering + MCP
  tts_node              ROS wrapper for speech
  tts_engine.py         Speech synthesis

## Roadmap

-   Phase 1: AI receptionist
-   Phase 2: Speech-to-Text
-   Phase 3: Raspberry Pi + SLAM + Nav2
-   Phase 4: Escort mode and voice conversations

## Notes

Performance and CPU utilization will be documented after testing on
Raspberry Pi 4. No estimated benchmark values are included.
