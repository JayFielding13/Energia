# Developer Onboarding - Rover Simulation

**Welcome to the Jetson Cube Orange Rover Simulation Project!**

This document provides a roadmap for new developers (especially those without physical hardware access) to get productive quickly.

---

## 🎯 Your Learning Path

### Phase 1: Environment Setup (2-3 hours)
**Goal:** Get the simulation running on your machine

1. ✅ Follow [GETTING_STARTED_GUIDE.md](GETTING_STARTED_GUIDE.md)
   - Install Ubuntu 22.04 and ROS2 Humble
   - Build the simulation workspace
   - Launch your first simulation

2. ✅ Verify everything works
   - Gazebo opens with rover model
   - RViz2 displays sensor data
   - You can control the rover with keyboard

**Success criteria:** You can drive the rover around in simulation

---

### Phase 2: ROS2 Fundamentals (1-2 weeks)
**Goal:** Understand the ROS2 ecosystem

1. 📚 Complete official ROS2 tutorials
   - [ROS2 Humble Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
   - Focus on: Topics, Publishers, Subscribers, Nodes

2. 🧪 Run the example scripts
   - [examples/sensor_monitor.py](examples/sensor_monitor.py) - Read sensor data
   - [examples/simple_avoid.py](examples/simple_avoid.py) - Basic autonomy

3. 🔧 Modify the examples
   - Change obstacle avoidance thresholds
   - Add LiDAR data to decision making
   - Subscribe to camera feed

**Success criteria:** You can write a ROS2 node that reads sensors and controls the rover

---

### Phase 3: Understanding Simulation vs Real (1 week)
**Goal:** Know what transfers to real hardware and what doesn't

1. 📖 Read [SIMULATION_ROVER_GAPS_ANALYSIS.md](SIMULATION_ROVER_GAPS_ANALYSIS.md)
   - Understand sensor topic compatibility
   - Learn about control architecture differences
   - Identify missing features (MAVLink, killswitch, etc.)

2. 🤔 Plan your project with portability in mind
   - Use ROS2 topics that match real rover
   - Avoid simulation-specific features
   - Write code that can run on both platforms

**Success criteria:** You understand what code will "just work" on real hardware

---

### Phase 4: Build Something Cool! (Ongoing)
**Goal:** Create autonomous behaviors or integrate AI

#### Project Ideas by Difficulty

**Beginner:**
- 🟢 Wall-following algorithm
- 🟢 Sensor data visualization dashboard
- 🟢 Waypoint navigation using GPS
- 🟢 Simple state machine for autonomous patrol

**Intermediate:**
- 🟡 Multi-sensor fusion for obstacle detection
- 🟡 Natural language control via LLM
- 🟡 Camera-based object detection
- 🟡 Dynamic obstacle avoidance

**Advanced:**
- 🔴 LLM-based autonomous mission planning
- 🔴 SLAM (Simultaneous Localization and Mapping)
- 🔴 Multi-robot coordination
- 🔴 Vision-language-action models for rover control

---

## 📚 Documentation Roadmap

Read these in order:

| Document | When to Read | Purpose |
|----------|-------------|---------|
| [README.md](README.md) | First | Overview of simulation environment |
| [GETTING_STARTED_GUIDE.md](GETTING_STARTED_GUIDE.md) | Day 1 | Complete setup instructions |
| [examples/README.md](examples/README.md) | Week 1 | Learn from working code |
| [SIMULATION_ROVER_GAPS_ANALYSIS.md](SIMULATION_ROVER_GAPS_ANALYSIS.md) | Week 2-3 | Understand sim vs real differences |
| [ros2_ws/src/energia_sim/README.md](ros2_ws/src/energia_sim/README.md) | As needed | Technical details about URDF, sensors, launch files |
| [ros2_ws/src/energia_sim/SENSORS.md](ros2_ws/src/energia_sim/SENSORS.md) | Week 2 | Deep dive into sensor specifications |
| [ros2_ws/src/energia_sim/TESTING_GUIDE.md](ros2_ws/src/energia_sim/TESTING_GUIDE.md) | Week 3+ | Advanced testing procedures |

---

## 🛠️ Development Workflow

### Typical Development Cycle

1. **Start Simulation**
   ```bash
   cd ~/rover_simulation/simulation/ros2_ws
   source install/setup.bash
   ros2 launch energia_sim full_simulation.launch.py
   ```

2. **Open New Terminal for Your Script**
   ```bash
   source /opt/ros/humble/setup.bash
   python3 ~/rover_simulation/simulation/examples/my_script.py
   ```

3. **Monitor Topics (Optional)**
   ```bash
   # In another terminal
   ros2 topic echo /ultrasonic/front
   ros2 topic hz /scan
   ```

4. **Iterate and Test**
   - Modify your script
   - Restart script (Ctrl+C, then run again)
   - Simulation keeps running - no need to restart

5. **Add Obstacles in Gazebo**
   - Click "Insert" tab
   - Drag models into the world
   - Test your obstacle avoidance

### Code Organization Tips

```
~/rover_simulation/
├── simulation/
│   ├── ros2_ws/           # Build workspace (don't edit directly)
│   └── examples/          # Official examples
└── my_projects/           # YOUR CODE GOES HERE
    ├── obstacle_avoidance/
    ├── llm_integration/
    └── waypoint_nav/
```

Keep your experimental code separate from the simulation package!

---

## 🤖 LLM Integration Quickstart

Since you're interested in AI/LLM integration, here's a fast-track:

### 1. Choose Your LLM Provider

**Cloud Options:**
- **Anthropic Claude** - Best for reasoning, strong safety
- **OpenAI GPT-4** - Popular, good docs
- **Google Gemini** - Multimodal (vision + text)

**Local Options:**
- **Ollama** - Easy local LLM deployment
- **llama.cpp** - Lightweight, fast
- **LM Studio** - GUI for local models

### 2. Install SDK

```bash
# For Anthropic Claude
pip3 install anthropic

# For OpenAI
pip3 install openai

# For local (Ollama)
pip3 install ollama
```

### 3. Start Simple

**Natural Language Control Example:**

```python
import anthropic
from geometry_msgs.msg import Twist

client = anthropic.Anthropic(api_key="YOUR_KEY")

def execute_nl_command(command: str, cmd_pub):
    """Convert natural language to robot command"""

    response = client.messages.create(
        model="claude-3-5-sonnet-20241022",
        max_tokens=256,
        messages=[{
            "role": "user",
            "content": f"""Convert this command to robot motion:
            "{command}"

            Reply with JSON: {{"linear_x": float, "angular_z": float}}
            linear_x: forward speed (m/s), positive = forward
            angular_z: rotation speed (rad/s), positive = left turn
            """
        }]
    )

    # Parse and execute
    import json
    cmd = json.loads(response.content[0].text)
    twist = Twist()
    twist.linear.x = cmd['linear_x']
    twist.angular.z = cmd['angular_z']
    cmd_pub.publish(twist)

# Usage in ROS2 node:
execute_nl_command("move forward slowly", cmd_vel_publisher)
execute_nl_command("turn left", cmd_vel_publisher)
execute_nl_command("stop", cmd_vel_publisher)
```

### 4. Advanced: Sensor-Aware Decision Making

Feed sensor data to LLM for contextual decisions:

```python
def llm_obstacle_decision(sensor_data: dict, client):
    """Ask LLM what to do based on sensors"""

    prompt = f"""You are controlling a robot. Sensor readings:
    - Front: {sensor_data['front']:.2f}m
    - Left: {sensor_data['left']:.2f}m
    - Right: {sensor_data['right']:.2f}m

    Decide action as JSON: {{"action": "forward/left/right/stop", "reason": "why"}}
    """

    response = client.messages.create(
        model="claude-3-5-sonnet-20241022",
        max_tokens=256,
        messages=[{"role": "user", "content": prompt}]
    )

    return json.loads(response.content[0].text)
```

### 5. Experiment!

- Try different prompting strategies
- Add memory (conversation history)
- Use vision models with camera feed
- Build autonomous mission planners

---

## 🚀 Quick Wins to Build Confidence

Start with these mini-projects to get comfortable:

### Project 1: Sensor Dashboard (1-2 hours)
Create a Python script that prints a live ASCII dashboard of all sensors.

**Skills learned:** ROS2 subscribers, multiple callbacks, formatting output

### Project 2: Follow the Wall (3-4 hours)
Use side ultrasonic sensors to follow a wall at constant distance.

**Skills learned:** PID control, continuous sensor processing, cmd_vel publishing

### Project 3: LLM Voice Control (4-6 hours)
Add speech-to-text, send to LLM, execute on rover.

**Skills learned:** External API integration, audio processing, command parsing

### Project 4: Camera Object Detection (6-8 hours)
Use OpenCV or LLM vision to detect objects, navigate towards them.

**Skills learned:** Image processing, vision integration, multi-sensor fusion

---

## 💡 Tips for Success

### Do's ✅
- **Start simple** - Get one sensor working before combining multiple
- **Use print statements** - ROS2 logging is your friend
- **Test incrementally** - Don't write 200 lines before testing
- **Read error messages** - They're usually helpful
- **Ask questions** - Documentation is your friend

### Don'ts ❌
- **Don't skip ROS2 tutorials** - Understanding topics/nodes is essential
- **Don't test untested code on real rover** - Simulation first, always
- **Don't hardcode paths** - Use ROS2 parameters and environment variables
- **Don't forget to source** - `source install/setup.bash` in every terminal
- **Don't panic** - Simulation can't break, experiment freely!

---

## 🆘 Getting Help

### When Something Doesn't Work

1. **Check terminal for errors** - Read the full error message
2. **Verify ROS2 is sourced** - `echo $ROS_DISTRO` should print "humble"
3. **List topics** - `ros2 topic list` shows what's available
4. **Check node status** - `ros2 node list` shows running nodes
5. **Google the error** - ROS2 community is huge, someone's solved it
6. **Check documentation** - See roadmap above

### Resources

- **ROS2 Docs:** https://docs.ros.org/en/humble/
- **Gazebo Tutorials:** https://classic.gazebosim.org/tutorials
- **ROS Discourse Forum:** https://discourse.ros.org/
- **Robotics Stack Exchange:** https://robotics.stackexchange.com/

### Project-Specific Help

- Review [SIMULATION_ROVER_GAPS_ANALYSIS.md](SIMULATION_ROVER_GAPS_ANALYSIS.md) for architecture questions
- Check [examples/](examples/) for working code patterns
- Read sensor specifications in [ros2_ws/src/energia_sim/SENSORS.md](ros2_ws/src/energia_sim/SENSORS.md)

---

## 🎓 Success Criteria

You're ready to work on real projects when you can:

- ✅ Launch simulation without help
- ✅ Write a ROS2 node from scratch
- ✅ Subscribe to any sensor topic and process data
- ✅ Publish velocity commands to move the rover
- ✅ Explain what will/won't transfer to real hardware
- ✅ Debug common ROS2 issues independently

---

## 🎯 Your First Week Plan

### Monday: Setup Day
- Install Ubuntu 22.04 + ROS2 Humble
- Clone repository
- Build workspace
- **Goal:** See rover in Gazebo

### Tuesday: ROS2 Basics
- Complete ROS2 beginner tutorials
- Run example scripts
- **Goal:** Understand topics and nodes

### Wednesday: Sensor Exploration
- Echo all sensor topics
- Visualize in RViz2
- Modify sensor_monitor.py
- **Goal:** Read any sensor confidently

### Thursday: Control Basics
- Study simple_avoid.py
- Write your own movement script
- Test in simulation
- **Goal:** Make rover do what you want

### Friday: Mini Project
- Choose a quick win project
- Build it from scratch
- **Goal:** Create something that works!

### Weekend: Experiment
- Try LLM integration
- Add camera processing
- Build something cool
- **Goal:** Have fun!

---

## 🎉 Welcome Aboard!

You're now equipped to:
- Set up the simulation environment
- Learn ROS2 fundamentals
- Write autonomous behaviors
- Integrate AI/LLMs
- Prepare code for real hardware deployment

**Remember:** The simulation is your safe playground. Break things, try wild ideas, and learn by doing!

**Next step:** Open [GETTING_STARTED_GUIDE.md](GETTING_STARTED_GUIDE.md) and start your setup!

**Happy coding!** 🤖🚀

---

**Questions?** Review the documentation roadmap above or ask for help on ROS Discourse.
