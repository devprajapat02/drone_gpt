# DroneGPT

## Introduction
DroneGPT is a pioneering project designed to revolutionize human-robot interactions by seamlessly integrating ChatGPT into the realm of robotics. The project boasts a flexible high-level function library that ensures ChatGPT's adaptability to a diverse range of tasks, form factors, and simulation environments. Showcasing its prowess through the Turtlesim and Tello drone simulator, ChatGPT demonstrates unparalleled flexibility and agility. Key technologies driving this innovation include OpenAI's GPT-3.5 Turbo for advanced language skills, the DroneBlocks Tello simulator, ROS (Robot Operating System) for seamless communication, Tkinter for a user-friendly speech-to-text interface, and the Vosk library for efficient voice-based communication. This amalgamation of cutting-edge technologies redefines the landscape of human-robot collaboration within the robotics field.

## Steps to Reproduce

### Setup

1. Clone the repository:
   ```
   git clone https://github.com/devprajapat02/drone_gpt.git
   ```
   into your ROS workspace.

2. Download `vosk-model-small-en-us-0.15` from [alphacephei.com](https://alphacephei.com/vosk/models) and move it to the "models" folder within the repository.

3. Install required dependencies:
   ```
   pip3 install -r requirements.txt
   ```

4. Install the pyaudio dependency:
   ```
   sudo apt install python3-pyaudio
   ```

5. Build the ROS workspace:
   ```
   cd <your_ros_workspace>
   catkin_make
   ```

6. Obtain a simulation key from [coding-sim.droneblocks.io](https://coding-sim.droneblocks.io/) and add it to the `config.json` file. Note: Use only the Chrome browser.

7. Add your OpenAI key to `./launch/example.launch` in the 'OPENAI_API_KEY' environment variable. Get an OpenAI key from [platform.openai.com](https://platform.openai.com/account/api-keys).

8. You're all set!

### Running the Program

1. Launch the program using the following command:
   ```
   roslaunch drone_gpt example.launch
   ```

2. Choose between CLI or Text-to-Speech input.

3. Provide the desired input, and the corresponding action will be performed in the simulation.

Feel free to contribute and explore the exciting possibilities of DroneGPT in enhancing human-robot collaboration within the robotics domain.
