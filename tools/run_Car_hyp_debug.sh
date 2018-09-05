for num_scenarios in 500; do
      dir="HypDespot_Car_Driving_Results_debug/n$num_scenarios";
      mkdir -p $dir;
      #for i in {1..200}; do
      echo "run HYP "-n $num_scenarios
      random_num=0 #$RANDOM
      cp ~/workspace/HypDESPOT-newrepo/catkin_ws/src/HyP_despot/Release/devel/lib/hyp_despot/hyp_despot_CarDriving ./hyp_despot_CarDriving
      ./hyp_despot_CarDriving --runs 1 -t 0.3 --discount 0.95 --prune 0.001 --xi 0.95 --simlen 400 -r $random_num -n $num_scenarios --GPU 1 --CPU 1 --GPUID 1 --num_thread 5 &> $dir/hyp_data_fixscn_0
      echo "End 0"

      scp panpan@flamingo.d1.comp.nus.edu.sg:~/Panpan-despot1/GPUDespot/Release/GPUDespot ./HypDespot_debug_fixsc_0
      ./HypDespot_debug_fixsc_0 --runs 1 -t 0.3 --discount 0.95 --prune 0.001 --xi 0.95 --simlen 400 -r $random_num -n $num_scenarios --GPU 1 --CPU 1 --GPUID 1 --num_thread 5 &> $dir/hyp_data_fixscn_0_ref
      echo "End 0 ref"

      # cp ~/workspace/HypDESPOT-newrepo/catkin_ws/src/HyP_despot/Release/devel/lib/hyp_despot/hyp_despot_CarDriving ./hyp_despot_CarDriving
      # ./hyp_despot_CarDriving --runs 1 -t 1 --discount 0.95 --prune 0.001 --xi 0.95 --simlen 21 -r $random_num -n $num_scenarios --GPU 1 --CPU 1 --GPUID 1 --num_thread 1 &> $dir/hyp_data_fixscn_1
      # echo "End 1"

      # scp panpan@flamingo.d1.comp.nus.edu.sg:~/Panpan-despot1/GPUDespot/Release/GPUDespot ./HypDespot_debug_fixsc_1
      # ./HypDespot_debug_fixsc_1 --runs 1 -t 1 --discount 0.95 --prune 0.001 --xi 0.95 --simlen 21 -r $random_num -n $num_scenarios --GPU 1 --CPU 1 --GPUID 1 --num_thread 1 &> $dir/hyp_data_fixscn_1_ref
      # echo "End 1 ref"

      # scp panpan@flamingo.d1.comp.nus.edu.sg:~/Panpan-despot1/GPUDespot/Release/GPUDespot ./HypDespot_debug_fixsc_2
      # ./HypDespot_debug_fixsc_2 --runs 1 -t 1 --discount 0.95 --prune 0.001 --xi 0.95 --simlen 100 -r $random_num -n $num_scenarios --GPU 1 --CPU 1 --GPUID 1 --num_thread 1 &> $dir/hyp_data_fixscn_2
      done;
      # dir="GPUDespot_Car_Driving_Results/n$num_scenarios";
      # mkdir -p $dir;
      # echo "run GPU "-n $num_scenarios
      # #for i in {1..200}; do
      # random_num=$RANDOM
     #  ./HypDespot --runs 200 --discount 0.983 --xi 0.95 --simlen 180 -r $random_num -n $num_scenarios --GPU 1 --CPU 0 --GPUID 3 &>> $dir/data$random_num
#done;
echo "End car driving experiments"

