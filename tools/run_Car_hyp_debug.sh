for num_scenarios in 500; do
      dir="HypDespot_Car_Driving_Results_debug/n$num_scenarios";
      mkdir -p $dir;
      for i in {1..200}; do
            echo "run HYP "-n $num_scenarios
            random_num=$RANDOM
            ./hyp_despot_CarDriving --runs 1 -t 0.3 --discount 0.95 --prune 0.001 --xi 0.95 --simlen 400 -r $random_num -n $num_scenarios --GPU 1 --CPU 1 --GPUID 1 --num_thread 5 &> $dir/hyp_data_$random_num
            echo "End "$i
      done;
done;
echo "End car driving experiments"

