### Autocar Firmware

------

An autonomous car firmware on Raspberry pi using `Orocos-RTT` toolchain and `PREEMPT_RT` patch.

For gazebo simulation, take a look at [this](https://github.com/parzival2/autocar) repository. 

##### Setup - PREEMPT_RT

The `PREEMPT_RT` patch has been applied by following the [instructions](https://lemariva.com/blog/2019/09/raspberry-pi-4b-preempt-rt-kernel-419y-performance-test) from here. When I applied the patch, I didn't notice a very big improvement in `cyclictest - sudo cyclictest --smp -p95 -m` results but actually they are bad compared to the stock kernel.

######  Stock Kernel

```
T: 0 (11011) P:95 I:1000 C:  26495 Min:      6 Act:   27 Avg:   17 Max:     138
T: 1 (11012) P:95 I:1500 C:  17654 Min:      6 Act:   16 Avg:   18 Max:     143
T: 2 (11013) P:95 I:2000 C:  13231 Min:      6 Act:   14 Avg:   15 Max:     131
T: 3 (11014) P:95 I:2500 C:  10582 Min:      6 Act:   18 Avg:   17 Max:     961
```

###### PREEMPT_RT

```
T: 0 ( 8918) P:95 I:1000 C:  22816 Min:      9 Act:   22 Avg:   22 Max:     146
T: 1 ( 8919) P:95 I:1500 C:  15187 Min:      8 Act:   25 Avg:   28 Max:     134
T: 2 ( 8920) P:95 I:2000 C:  11373 Min:      9 Act:   23 Avg:   24 Max:     103
T: 3 ( 8921) P:95 I:2500 C:   9084 Min:     11 Act:   26 Avg:   26 Max:     133
```

Still it has to be analyzed what might be problem. With stock kernel, it should give nominal performance without any problems. 

The above mentioned site also have some pre-compiled images in their github account which should function out of the box and can be used without compiling.

##### Setup - ROS cross compile

I was able to cross-compile normal c++ projects but was not able to do it for ROS. Compiling ros and especially `orocos` on Raspberry pi is not possible as it will take huge amount of ram. I tried it using an external flash drive as ram but it was very very slow. 

So I used the approach [here](https://github.com/ryankurte/docker-rpi-emu) and setup a docker environment which emulates raspberry pi and since the directory structure would be same you can just copy and paste the `/opt/ros` folder from your docker container to Raspberry pi(thats what i did and worked without any problems.). It is a bit slow compared to the speeds you get when you compile normally but it is lot faster compared to Raspberry pi.



> Note : Still under development.