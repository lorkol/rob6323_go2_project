# ROB6323 Go2 Project — Isaac Lab

This repository is the starter code for the NYU Reinforcement Learning and Optimal Control project in which students train a Unitree Go2 walking policy in Isaac Lab starting from a minimal baseline and improve it via reward shaping and robustness strategies. Please read this README fully before starting and follow the exact workflow and naming rules below to ensure your runs integrate correctly with the cluster scripts and grading pipeline.

## Repository policy

- Fork this repository and do not change the repository name in your fork.  
- Your fork must be named rob6323_go2_project so cluster scripts and paths work without modification.

### Prerequisites

- **GitHub Account:** You must have a GitHub account to fork this repository and manage your code. If you do not have one, [sign up here](https://github.com/join).

### Links
1.  **Project Webpage:** [https://machines-in-motion.github.io/RL_class_go2_project/](https://machines-in-motion.github.io/RL_class_go2_project/)
2.  **Project Tutorial:** [https://github.com/machines-in-motion/rob6323_go2_project/blob/master/tutorial/tutorial.md](https://github.com/machines-in-motion/rob6323_go2_project/blob/master/tutorial/tutorial.md)

## Connect to Greene

- Connect to the NYU Greene HPC via SSH; if you are off-campus or not on NYU Wi‑Fi, you must connect through the NYU VPN before SSHing to Greene.  
- The official instructions include example SSH config snippets and commands for greene.hpc.nyu.edu and dtn.hpc.nyu.edu as well as VPN and gateway options: https://sites.google.com/nyu.edu/nyu-hpc/accessing-hpc?authuser=0#h.7t97br4zzvip.

## Clone in $HOME

After logging into Greene, `cd` into your home directory (`cd $HOME`). You must clone your fork into `$HOME` only (not scratch or archive). This ensures subsequent scripts and paths resolve correctly on the cluster. Since this is a private repository, you need to authenticate with GitHub. You have two options:

### Option A: Via VS Code (Recommended)
The easiest way to avoid managing keys manually is to configure **VS Code Remote SSH**. If set up correctly, VS Code forwards your local credentials to the cluster.
- Follow the [NYU HPC VS Code guide](https://sites.google.com/nyu.edu/nyu-hpc/training-support/general-hpc-topics/vs-code) to set up the connection.

> **Tip:** Once connected to Greene in VS Code, you can clone directly without using the terminal:
> 1. **Sign in to GitHub:** Click the "Accounts" icon (user profile picture) in the bottom-left sidebar. If you aren't signed in, click **"Sign in with GitHub"** and follow the browser prompts to authorize VS Code.
> 2. **Clone the Repo:** Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`), type **Git: Clone**, and select it.
> 3. **Select Destination:** When prompted, select your home directory (`/home/<netid>/`) as the clone location.
>
> For more details, see the [VS Code Version Control Documentation](https://code.visualstudio.com/docs/sourcecontrol/intro-to-git#_clone-a-repository-locally).

### Option B: Manual SSH Key Setup
If you prefer using a standard terminal, you must generate a unique SSH key on the Greene cluster and add it to your GitHub account:
1. **Generate a key:** Run the `ssh-keygen` command on Greene (follow the official [GitHub documentation on generating a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent#generating-a-new-ssh-key)).
2. **Add the key to GitHub:** Copy the output of your public key (e.g., `cat ~/.ssh/id_ed25519.pub`) and add it to your account settings (follow the [GitHub documentation on adding a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)).

### Execute the Clone
Once authenticated, run the following commands. Replace `<your-git-ssh-url>` with the SSH URL of your fork (e.g., `git@github.com:YOUR_USERNAME/rob6323_go2_project.git`).
```
cd $HOME
git clone <your-git-ssh-url> rob6323_go2_project
```
*Note: You must ensure the target directory is named exactly `rob6323_go2_project`. This ensures subsequent scripts and paths resolve correctly on the cluster.*
## Install environment

- Enter the project directory and run the installer to set up required dependencies and cluster-side tooling.  
```
cd $HOME/rob6323_go2_project
./install.sh
```
Do not skip this step, as it configures the environment expected by the training and evaluation scripts. It will launch a job in burst to set up things and clone the IsaacLab repo inside your greene storage. You must wait until the job in burst is complete before launching your first training. To check the progress of the job, you can run `ssh burst "squeue -u $USER"`, and the job should disappear from there once it's completed. It takes around **30 minutes** to complete. 
You should see something similar to the screenshot below (captured from Greene):

![Example burst squeue output](docs/img/burst_squeue_example.png)

In this output, the **ST** (state) column indicates the job status:
- `PD` = pending in the queue (waiting for resources).
- `CF` = instance is being configured.
- `R`  = job is running.

On burst, it is common for an instance to fail to configure; in that case, the provided scripts automatically relaunch the job when this happens, so you usually only need to wait until the job finishes successfully and no longer appears in `squeue`.

## What to edit

- In this project you'll only have to modify the two files below, which define the Isaac Lab task and its configuration (including PPO hyperparameters).  
  - source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env.py  
  - source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env_cfg.py
PPO hyperparameters are defined in source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/agents/rsl_rl_ppo_cfg.py, but you shouldn't need to modify them.

## How to edit

- Option A (recommended): Use VS Code Remote SSH from your laptop to edit files on Greene; follow the NYU HPC VS Code guide and connect to a compute node as instructed (VPN required off‑campus) (https://sites.google.com/nyu.edu/nyu-hpc/training-support/general-hpc-topics/vs-code). If you set it correctly, it makes the login process easier, among other things, e.g., cloning a private repo.
- Option B: Edit directly on Greene using a terminal editor such as nano.  
```
nano source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env.py
```
- Option C: Develop locally on your machine, push to your fork, then pull changes on Greene within your $HOME/rob6323_go2_project clone.

> **Tip:** Don't forget to regularly push your work to github

## Launch training

- From $HOME/rob6323_go2_project on Greene, submit a training job via the provided script.  
```
cd "$HOME/rob6323_go2_project"
./train.sh
```
- Check job status with SLURM using squeue on the burst head node as shown below.  
```
ssh burst "squeue -u $USER"
```
Be aware that jobs can be canceled and requeued by the scheduler or underlying provider policies when higher-priority work preempts your resources, which is normal behavior on shared clusters using preemptible partitions.

## Where to find results

- When a job completes, logs are written under logs in your project clone on Greene in the structure logs/[job_id]/rsl_rl/go2_flat_direct/[date_time]/.  
- Inside each run directory you will find a TensorBoard events file (events.out.tfevents...), neural network checkpoints (model_[epoch].pt), YAML files with the exact PPO and environment parameters, and a rollout video under videos/play/ that showcases the trained policy.  

## Download logs to your computer

Use `rsync` to copy results from the cluster to your local machine. It is faster and can resume interrupted transfers. Run this on your machine (NOT on Greene):

```
rsync -avzP -e 'ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null' <netid>@dtn.hpc.nyu.edu:/home/<netid>/rob6323_go2_project/logs ./
```

*Explanation of flags:*
- `-a`: Archive mode (preserves permissions, times, and recursive).
- `-v`: Verbose output.
- `-z`: Compresses data during transfer (faster over network).
- `-P`: Shows progress bar and allows resuming partial transfers.

## Visualize with TensorBoard

You can inspect training metrics (reward curves, loss values, episode lengths) using TensorBoard. This requires installing it on your local machine.

1.  **Install TensorBoard:**
    On your local computer (do NOT run this on Greene), install the package:
    ```
    pip install tensorboard
    ```

2.  **Launch the Server:**
    Navigate to the folder where you downloaded your logs and start the server:
    ```
    # Assuming you are in the directory containing the 'logs' folder
    tensorboard --logdir ./logs
    ```

3.  **View Metrics:**
    Open your browser to the URL shown (usually `http://localhost:6006/`).

## Debugging on Burst

Burst storage is accessible only from a job running on burst, not from the burst login node. The provided scripts do not automatically synchronize error logs back to your home directory on Greene. However, you will need access to these logs to debug failed jobs. These error logs differ from the logs in the previous section.

The suggested way to inspect these logs is via the Open OnDemand web interface:

1.  Navigate to [https://ood-burst-001.hpc.nyu.edu](https://ood-burst-001.hpc.nyu.edu).
2.  Select **Files** > **Home Directory** from the top menu.
3.  You will see a list of files, including your `.err` log files.
4.  Click on any `.err` file to view its content directly in the browser.

> **Important:** Do not modify anything inside the `rob6323_go2_project` folder on burst storage. This directory is managed by the job scripts, and manual changes may cause synchronization issues or job failures.

## Project scope reminder

- The assignment expects you to go beyond velocity tracking by adding principled reward terms (posture stabilization, foot clearance, slip minimization, smooth actions, contact and collision penalties), robustness via domain randomization, and clear benchmarking metrics for evaluation as described in the course guidelines.  
- Keep your repository organized, document your changes in the README, and ensure your scripts are reproducible, as these factors are part of grading alongside policy quality and the short demo video deliverable.

## Resources

- [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/main/source/setup/ecosystem.html) — Everything you need to know about IsaacLab, and more!
- [Isaac Lab ANYmal C environment](https://github.com/isaac-sim/IsaacLab/tree/main/source/isaaclab_tasks/isaaclab_tasks/direct/anymal_c) — This targets ANYmal C (not Unitree Go2), so use it as a reference and adapt robot config, assets, and reward to Go2.
- [DMO (IsaacGym) Go2 walking project page](https://machines-in-motion.github.io/DMO/) • [Go2 walking environment used by the authors](https://github.com/Jogima-cyber/IsaacGymEnvs/blob/e351da69e05e0433e746cef0537b50924fd9fdbf/isaacgymenvs/tasks/go2_terrain.py) • [Config file used by the authors](https://github.com/Jogima-cyber/IsaacGymEnvs/blob/e351da69e05e0433e746cef0537b50924fd9fdbf/isaacgymenvs/cfg/task/Go2Terrain.yaml) — Look at the function `compute_reward_CaT` (beware that some reward terms have a weight of 0 and thus are deactivated, check weights in the config file); this implementation includes strong reward shaping, domain randomization, and training disturbances for robust sim‑to‑real, but it is written for legacy IsaacGym and the challenge is to re-implement it in Isaac Lab.
- **API References**:
    - [ArticulationData (`robot.data`)](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.assets.html#isaaclab.assets.ArticulationData) — Contains `root_pos_w`, `joint_pos`, `projected_gravity_b`, etc.
    - [ContactSensorData (`_contact_sensor.data`)](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.sensors.html#isaaclab.sensors.ContactSensorData) — Contains `net_forces_w` (contact forces).

---
Students should only edit README.md below this ligne.



# Rob6323Go2 Environment

This project implements a custom reinforcement learning (RL) environment for quadruped locomotion, based on the IsaacLab framework. The main logic is contained in two files:

- `rob6323_go2_env.py`: The environment class and reward logic
- `rob6323_go2_env_cfg.py`: The configuration for simulation, robot, and reward shaping

## Environment Implementation (`rob6323_go2_env.py`)

The `Rob6323Go2Env` class extends `DirectRLEnv` and provides a highly customized simulation for the Unitree Go2 robot. Key features include:

- **Action and Command Handling**: Actions are interpreted as joint position deviations, and commands specify target linear and angular velocities.
- **PD Control**: Implements explicit PD control for joint torques, with friction effects (viscous and stiction) added for realism. Gains and limits are configurable.
- **Reward Shaping**: The reward function is carefully designed to encourage stable, natural locomotion. It includes:
    - Linear and angular velocity tracking (exponential penalties)
    - Action rate penalization (first and second derivatives)
    - Raibert heuristic for gait timing and contact scheduling
    - Penalties for non-flat orientation, vertical movement, excessive joint velocities, and body roll/pitch
    - Foot clearance and contact force shaping for smooth stepping
- **Gait and Contact Scheduling**: Uses a Raibert-inspired heuristic to generate periodic gait patterns, with smooth transitions between stance and swing phases for each foot.
- **Observation and Reset Logic**: Observations include robot state and gait phase information. The reset logic randomizes initial conditions and friction parameters for robustness.
- **Debug Visualization**: Optional visualization of velocity commands and robot state for easier debugging and analysis.

## Configuration (`rob6323_go2_env_cfg.py`)

The configuration file defines all simulation, robot, and reward parameters:

- **Simulation Settings**: Time step, physics material properties, and terrain configuration.
- **Robot Configuration**: Uses the Unitree Go2 model, with actuators grouped for the legs. Explicitly disables implicit P/D gains in favor of the environment's PD controller.
- **PD Gains and Limits**: Proportional and derivative gains, as well as torque limits, are set here for easy tuning.
- **Reward Scales**: All reward and penalty terms are configurable, allowing for rapid experimentation with different shaping strategies.
- **Gait and Foot Clearance**: Parameters for desired foot clearance and contact force shaping are included to promote natural stepping.

## Design Choices

- **Explicit PD Control**: By disabling implicit gains and implementing explicit PD control, the environment allows for more direct and interpretable tuning of robot behavior.
- **Reward Shaping**: The reward function is modular and extensible, supporting both basic tracking and advanced gait shaping (Raibert heuristic, foot clearance, contact forces).
- **Robustness**: Randomization of friction and initial conditions during resets helps the policy generalize to a variety of scenarios.
- **Extensibility**: The environment and config are structured for easy modification, supporting new reward terms, robot models, or terrain types.

## Implementing additions to the Baseline

### Part 1: Adding Action Rate Penalties
Smooth motion requires penalizing jerky actions. To do this, we need to track the history of actions taken by the policy.

We added the variable action_rate_reward_scale  to the config file, setting it to =-0.1
In the env file, we added to the __init__() method when creating the _episode_sums dictionary the keys rew_action_rate and raibert_heuristic.
When an environment resets, we must clear this history so the new episode starts fresh, so in the _reset_idx() we set self.last_actions[env_ids] = 0.

We updated the _get_rewards(): We calculate the "rate" (first derivative) and "acceleration" (second derivative) of the actions to penalize high-frequency oscillations.

### Part 2: Implementing a Low-Level PD Controller
Instead of relying on the physics engine's implicit PD controller, we will implement our own torque-level control. This gives us full control over the gains and limits.


In the config file, we disabled the built-in PD controller in the config and defined our custom gains.
In the env __init__(): we load the Kp, Kd gains into tensors for efficient computation.
We calculate the torques manually using the standard PD formula

### Part 3: Early Stopping (Min Base Height)
To speed up training, we should terminate episodes early if the robot falls down or collapses. It will also help learning that the base should stay elevated.

In the config file, we Define the threshold for termination: base_height_min  = 0.2
In the Env file, in _get_dones() we check the robot's base height (z-coordinate) against the threshold.

### Part 4: Raibert Heuristic (Gait Shaping)
The Raibert Heuristic is a classic control strategy that places feet to stabilize velocity. We will use it as a "teacher" reward to encourage the policy to learn proper stepping. 

In the config file we define the reward scales and increase observation space to include clock inputs (4 phases).

In the env file we track the "phase" of the gait and identify feet bodies.

We define Foot Indices Helper We need to know which body indices correspond to the feet to get their positions.

We implement Gait Logic.
We implement a function that advances the gait clock and calculates where the feet should be based on the command velocity. We also need to reset the gait index on episode reset.

We implement Raibert Reward: We calculate the error between where the foot IS and where the Raibert Heuristic says it SHOULD be.

Finally, we integrate into Observations and Rewards. We expose the clock inputs to the policy and add the reward term.


### Part 5: Refining the Reward Function

To achieve stable and natural-looking locomotion, we need to shape the robot's behavior further. We added penalties for:
Non-flat body orientation (projected gravity).
Vertical body movement (bouncing).
Excessive joint velocities.
Body rolling and pitching (angular velocity).

In out config file, we added the following reward scales:
orient_reward_scale = -5.0
lin_vel_z_reward_scale = -0.02
dof_vel_reward_scale = -0.0001
ang_vel_xy_reward_scale = -0.001

Finally, we implemented the logic to calculate these rewards inside _get_rewards


### Part 6: Advanced Foot Interaction

We added critical rewards for legged locomotion: Foot Clearance (lifting feet during swing) and Contact Forces (grounding feet during stance).
In the config file we added the following scales:
feet_clearance_reward_scale = -30.0
tracking_contacts_shaped_force_reward_scale = 4.0



## Extra Credits 1: Friction
To transfer learned policies to real robot, it is necessary to create a more realistic simulation, especially add actuator friction
dynamics. In this task, we needed to add a simple actuator friction/viscous model in our
environment code and randomize parameters per episode This friction model combines static and viscous friction and should be subtracted from the torques
computed by our low-level PD controller before they are sent to the robot.

τ_stiction = Fs · tanh(q˙/0.1)
τ_viscous = µv · q˙
τ_friction = τ_stiction + τviscous
τ_PD ← τ_PD − τfriction
Where:
τ_PD is the output torque from the PD controller

q˙ is the joint velocity

Fs is the stiction coefficient

µv is the viscous coefficient.

µv ∼ U(0.0, 0.3), Fs ∼ U(0.0, 2.5)

We also added this randomization logic to our episode reset

This forces the policy to overcome internal joint resistance, significantly narrowing the sim-to-real gap. This robustness is crucial for realworld deployment

Every time before sending a command of torque to the actuator, the model add this random friction to the signal