      const N=par.gear_ratio[i],eta=par.gear_efficiency[i],kt=par.torque_constant_Nm_A[i],R=par.resistance_ohm[i],L=par.inductance_H[i],V=par.bus_voltage_V[i]*par.max_pwm_duty[i];
      const limit=par.peak_current_A[i]*o.torqueScale,speed=N*qd[i],gearLimit=par.gear_output_torque_limit_Nm[i]*o.torqueScale;
      const transition=Math.tanh(speed/par.gear_efficiency_transition_motor_rad_s[i]);
      const beltLimit=par.belt_allowable_output_torque_Nm?.[i]??Infinity;
      const transmittedLimit=Math.min(gearLimit,beltLimit);
      if(Math.abs(tau)>beltLimit)this.monitor.beltDemandLimitSeconds[i]+=dt;
      if(Math.abs(tau)>gearLimit)this.monitor.gearDemandLimitSeconds[i]+=dt;
      const demand=clip(tau,-transmittedLimit,transmittedLimit),eff=Math.exp(Math.log(eta)*transition*Math.sign(demand));
      const requested=clip(demand/(N*eff*kt),-limit,limit),emf=kt*speed;
      this.voltage[i]=clip(R*requested+emf,-V,V);
      if(!this.motorPower){this.voltage[i]=0;this.current[i]=0;}
      else{const equilibrium=(this.voltage[i]-emf)/R;this.current[i]=equilibrium+(this.current[i]-equilibrium)*Math.exp(-R*dt/L);}
      const rotor=kt*this.current[i],effActual=Math.exp(Math.log(eta)*transition*Math.sign(rotor));
      this.torque[i]=N*rotor*effActual;ctrl[i]=this.torque[i];
      const copper=this.current[i]*this.current[i]*R,loss=rotor*speed-this.torque[i]*qd[i];
      this.temperature[i]+=(copper-(this.temperature[i]-25)/par.thermal_resistance_K_W[i])*dt/par.thermal_capacity_J_K[i];
      this.electricalEnergy[i]+=this.voltage[i]*this.current[i]*dt;this.copperHeat[i]+=copper*dt;
      this.mechanicalEnergy[i]+=this.torque[i]*qd[i]*dt;this.gearHeat[i]+=loss*dt;