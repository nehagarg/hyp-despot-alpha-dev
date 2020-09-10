# HyP-DESPOT-ALPHA

This is the source code of Hyp-DESPOT-ALPHA algorithm published in RSS 2019 paper:

Neha Priyadarshini Garg, David Hsu, Wee Sun Lee : DESPOT-Alpha: Online POMDP Planning with Large State and Observation Spaces. Robotics: Science & Systems XV 2019  [PDF](http://www.roboticsproceedings.org/rss15/p06.pdf).

The code was tested on ubuntu 16 with cuda 9 with GTX1080 GPUs. To run this code for for problems mentioned the paper:

1. Clone HyP-Despot-LargeObservation-Without-Grasping branch for the ubuntu 16 code :
```git clone https://github.com/AdaCompNUS/HyP-DESPOT-Release.git -b HyP-Despot-LargeObservation-Without-Grasping
```
For ubuntu 18 and cuda 11, use master branch. Though ubuntu 18 and cuda 11 version hasn't been tested.
2. Compile the code :
```mkdir build
   cd build
   cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Release ../
   make
```
3. Run the code :

* [TigerI](src/HyP_examples/tiger/src) CPU only
For Despot
```
./devel/lib/hyp_despot/hyp_despot_tiger -v 3 -t 1 -n 500
```
For Despot-ALPHA
```
./devel/lib/hyp_despot/hyp_despot_tiger -v 3 -t 1 -n 100 --nobs 10 --solver BTDESPOTALPHAST
```
* [Danger Tag](src/HyP_examples/tag/src) CPU only
For Despot
```
./devel/lib/hyp_despot/hyp_despot_danger_tag -v 3 -m ../src/HyP_examples/tag/config_files/danger_tag/openSingleR14x14-movementError6.txt -l DANGERTAG -t 1 -n 1600

```
For Despot-ALPHA
```
./devel/lib/hyp_despot/hyp_despot_danger_tag -v 3 -m ../src/HyP_examples/tag/config_files/danger_tag/openSingleR14x14-movementError6.txt -l DANGERTAG -t 1 -n 50 --nobs 10 --solver=BTDESPOTALPHAST

```
* [Navigation in unknown environment](src/HyP_examples/unknown_navigation/src)
For Despot
```
./devel/lib/hyp_despot/hyp_despot_unknav -v 3 -m ../src/HyP_examples/unkown_navigation/config_files/new_belief_16bits_02noise.txt -n 100 --CPU 1 --num_thread 10 --GPU 1 --GPUID <GPUID> -t 1 -l RANDOM -d 60

```
For Despot-ALPHA
```
./devel/lib/hyp_despot/hyp_despot_unknav -v 3 -m ../src/HyP_examples/unkown_navigation/config_files/new_belief_16bits_02noise.txt -d 60 -n 100 --nobs 10 --CPU 1 --num_thread 10 --GPU 1 --GPUID <GPUID> -t 1 -l RANDOM  --solver=BTDESPOTALPHAST

```
Set --CPU and --GPU to 0 for non-parallel version
For navigation with information gathering actions, give ../src/HyP_examples/unkown_navigation/config_files/new_belief_16bits_04noise_stay002.txt as argument to -m

* [Multi-agent RockSample](src/HyP_examples/ma_rock_sample/src)

For Despot
```
./devel/lib/hyp_despot/hyp_despot_mars -v 3 -m ../src/HyP_examples/ma_rock_sample/config_files/continuous_obs_2agents.txt --CPU 1 --num_thread 10 --GPU 1 --GPUID <GPUID> -n 50  -t 1

```
For Despot-ALPHA
```
./devel/lib/hyp_despot/hyp_despot_mars -v 3 -m ../src/HyP_examples/ma_rock_sample/config_files/continuous_obs_2agents.txt --CPU 1 --num_thread 10 --GPU 1 --GPUID <GPUID> --nobs 10 -n 50  -t 1 --solver=BTDESPOTALPHAST

```
Set --CPU and --GPU to 0 for non-parallel version
For single agent, give ../src/HyP_examples/ma_rock_sample/config_files/continuous_obs.txt as argument to -m . For discrete obsservation with 1 agent, give ../src/HyP_examples/ma_rock_sample/config_files/1_agent.txt to -m. If -m is not specified, by default discrete observation and 2 agents will be used. --size can be used to specify size of the square grid and --number to specify number of rocks. Default size is 15 and default number of rocks is 15.



* [Autonomous driving in a crowd](src/HyP_examples/CarDriving/src)
For Despot
```
./devel/lib/hyp_despot/hyp_despot_CarDriving -v 3 -s 200 -t 0.1 -n 1000 --max-policy-simlen 90 --GPUID <GPUID>

```
For Despot-ALPHA
```
./devel/lib/hyp_despot/hyp_despot_CarDriving -v 3 -s 200 -t 0.1 --nobs 10 -n 1000 --max-policy-simlen 6 -v 3 --GPUID <GPUID>  --econst 0.95 --oeconst 0.01 -l DONOTHING -d 10 --solver=BTDESPOTALPHAST

```
Set --CPU and --GPU to 0 for non-parallel version. Also set -n to 200 for DESPOT and to 100 for DESPOT-ALPHA for non parallel version.


The code is based on [HyP-DESPOT package](https://github.com/AdaCompNUS/hyp-despot) which closely follows the [API](https://github.com/AdaCompNUS/despot/tree/API_redesign/doc) in [the DESPOT package](https://github.com/AdaCompNUS/despot).

To use DESPOT-ALPHA for your own problem, a POMDP model needs to be created. For model not using parallelization, DESPOT package [tutorial](https://github.com/AdaCompNUS/despot/blob/API_redesign/doc/cpp_model_doc/Tutorial%20on%20Using%20DESPOT%20with%20cpp%20model.md) needs to followed. In addition, dummy GPU functions need to be declared for code compilation. See [tiger](src/HyP_examples/tiger/src/tiger.h) problem for example.

For model using parallelization, additional GPU model needs to be built as described in this [doc](doc/Build_GPU_POMDP_model_with_CUDA.md) for HyP-Despot. For HyP-Despot-Alpha, apart from this,

1) Specify one additional function
```
DEVICE float Dvc_ObsProb(OBS_TYPE& obs, Dvc_State& state, int action)
```
in Dvc_DSPOMDP model. For example see defintion in src/HyP_examples/unkown_navigation/src/GPU_Unk_nav/GPU_UncNavigation.cu This function should be linked to global function pointer DvcModelObsProb_ like in For example see  src/HyP_examples/unkown_navigation/src/GPU_Unk_nav/main.cu

2) Specify one additional function
```
Dvc_State* GetPointerToParticleList(int offset, Dvc_State* full_list) const;
```
in DSPOMDP model. For example see  src/HyP_examples/unkown_navigation/src/Unc_Navigation/UncNavigation.h and src/HyP_examples/unkown_navigation/src/GPU_Unk_nav/GPU_UncNavigation.cu

3) Change the GPU memory allocation size in
```Dvc_State* AllocGPUParticles(int numParticles, MEMORY_MODE mode, Dvc_State*** particles_for_all_actions) const
```
function. See the [commit](https://github.com/nehagarg/hyp-despot-alpha-dev/commit/0ae03f5a87dc8cedd534d5af225ff12e45aa87e5) for example.

```
