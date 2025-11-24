# Install
Create a virtual environment with Python 3.10 and activate it, e.g. with [`miniconda`](https://docs.anaconda.com/free/miniconda/index.html):
```bash
conda create -y -n lerobot python=3.10
conda activate lerobot
```

Install 🤗 LeRobot:
```bash
pip install -e . -i https://pypi.tuna.tsinghua.edu.cn/simple

pip uninstall numpy
pip install numpy==1.26.0
pip install pynput
```

/!\ For Linux only, ffmpeg and opencv requires conda install for now. Run this exact sequence of commands:
```bash
conda install -c conda-forge ffmpeg
pip uninstall opencv-python
conda install "opencv>=4.10.0"
```

Install Piper:  
```bash
pip install python-can
pip install piper_sdk
sudo apt update && sudo apt install can-utils ethtool
pip install pygame
```

# piper集成lerobot
见lerobot_piper_tutorial/1. 🤗 LeRobot：新增机械臂的一般流程.pdf

# Teleoperate
```bash
cd piper_scripts/
bash can_activate.sh can0 1000000

cd ..
python lerobot/scripts/control_robot.py \
    --robot.type=piper \
    --robot.inference_time=false \
    --control.type=teleoperate
```

# Record
Set dataset root path
```bash
HF_USER=$PWD/data
echo $HF_USER
```

```bash
python lerobot/scripts/control_robot.py \
    --robot.type=piper \
    --robot.inference_time=false \
    --control.type=record \
    --control.fps=30 \
    --control.single_task="move" \
    --control.repo_id=${HF_USER}/test \
    --control.num_episodes=2 \
    --control.warmup_time_s=2 \
    --control.episode_time_s=10 \
    --control.reset_time_s=10 \
    --control.play_sounds=true \
    --control.push_to_hub=false
```

Press right arrow -> at any time during episode recording to early stop and go to resetting. Same during resetting, to early stop and to go to the next episode recording.  
Press left arrow <- at any time during episode recording or resetting to early stop, cancel the current episode, and re-record it.  
Press escape ESC at any time during episode recording to end the session early and go straight to video encoding and dataset uploading.  

# visualize
```bash
python lerobot/scripts/visualize_dataset.py \
    --repo-id ${HF_USER}/test \
    --episode-index 0
```

# Replay
```bash
python lerobot/scripts/control_robot.py \
    --robot.type=piper \
    --robot.inference_time=false \
    --control.type=replay \
    --control.fps=30 \
    --control.repo_id=${HF_USER}/test \
    --control.episode=0
```

# Caution

1. In lerobots/common/datasets/video_utils, the vcodec is set to **libopenh264**, please find your vcodec by **ffmpeg -codecs**


# Train
具体的训练流程见lerobot_piper_tutorial/2. 🤗 AutoDL训练.pdf
```bash
python lerobot/scripts/train.py \
  --dataset.repo_id=${HF_USER}/jack \
  --policy.type=act \
  --output_dir=outputs/train/act_jack \
  --job_name=act_jack \
  --device=cuda \
  --wandb.enable=true
``` 


# Inference
还是使用control_robot.py中的record loop，配置 **--robot.inference_time=true** 可以将手柄移出。
```bash
python lerobot/scripts/control_robot.py \
    --robot.type=piper \
    --robot.inference_time=true \
    --control.type=record \
    --control.fps=30 \
    --control.single_task="move" \
    --control.repo_id=$USER/eval_act_jack \
    --control.num_episodes=1 \
    --control.warmup_time_s=2 \
    --control.episode_time_s=30 \
    --control.reset_time_s=10 \
    --control.push_to_hub=false \
    --control.policy.path=outputs/train/act_koch_pick_place_lego/checkpoints/latest/pretrained_model
```

遥操作测试
```
python lerobot/scripts/control_robot.py     --robot.type=piper     --robot.inference_time=false     --control.type=teleoperate
```

采集数据
```
python lerobot/scripts/control_robot.py     --robot.type=piper     --robot.inference_time=false     --control.type=record     --control.fps=30     --control.single_task="move"     --control.repo_id=${HF_USER}/test     --control.num_episodes=10     --control.warmup_time_s=15     --control.episode_time_s=60     --control.reset_time_s=50     --control.play_sounds=true     --control.push_to_hub=false  --control.resume=true
```

测试结果
```
python lerobot/scripts/control_robot.py     --robot.type=piper     --robot.inference_time=true     --control.type=record     --control.fps=30     --control.single_task="move"     --control.num_episodes=1     --control.warmup_time_s=3     --control.episode_time_s=60     --control.reset_time_s=10     --control.push_to_hub=false     --control.policy.path=pretrained_model --control.repo_id=$USER/eval_test105
```