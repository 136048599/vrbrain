/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_circle.pde - init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
static bool circle_init(bool ignore_checks)
{
	uint32_t dist;

    if ((GPS_ok() && inertial_nav.position_ok()) || ignore_checks) {
        circle_pilot_yaw_override = false;

        // initialize speeds and accelerations
        pos_control.set_speed_xy(wp_nav.get_speed_xy());
        pos_control.set_accel_xy(wp_nav.get_wp_acceleration());
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        if(g.ch7_option == AUX_SWITCH_SET_CIRCLE_CNTR) {
        	circle_aux_chan = 7;
        } else if (g.ch8_option == AUX_SWITCH_SET_CIRCLE_CNTR) {
        	circle_aux_chan = 8;
        } else {
        	circle_aux_chan = 0;
        }

        Vector3f cur_loc = pv_location_to_vector(current_loc);

        //calculate distance from center
        if(circle_aux_chan != 0) {
        	dist = pythagorous2(cur_loc.x - circle_nav.get_center().x , cur_loc.y - circle_nav.get_center().y );
        }

        if (read_circle_chan() == AUX_SWITCH_MIDDLE) {
        	//if AUX channel pos is in middle then get distance from saved center to calculate circle radius
            circle_nav.set_radius(dist);

       } else if (read_circle_chan() == AUX_SWITCH_HIGH) {
    	   // for users with two way switch and to avoid diving in the center
    	   // of the circle for panorama we check if distance is over 1,5 meter before setting _radius to 0
    	   // Otherwise we use distance as a radius.
        	if(dist > 150.0f) {
        		circle_nav.set_radius(dist);
        	} else {
            	// if AUX channel pos is in HIGH the sets the _radius to 0 for panorama shots.
                circle_nav.set_radius(0.0f);
        	}
        }

        // initialise circle controller including setting the circle center based on vehicle speed
        if(ap.circle_center_set) {
            circle_nav.init(circle_nav.get_center());
        } else {
            circle_nav.init();
        }

        return true;
    }else{
        return false;
    }
}

static uint8_t read_circle_chan() {

	if(circle_aux_chan == 7) {
		return read_3pos_switch(g.rc_7.radio_in);
	} else if (circle_aux_chan == 8) {
		return read_3pos_switch(g.rc_8.radio_in);
	} else {
		return 0;
	}

}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
static void circle_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete) {
        // To-Do: add some initialisation of position controllers
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
        if (target_yaw_rate != 0) {
            circle_pilot_yaw_override = true;
        }

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }


        //read pitch channel and use it to update circle radius
        // To-Do: make variable increase based on stick position
        if(g.rc_2.control_in > 0 && circle_nav.get_radius() < 10000.0f) {
        	circle_nav.set_radius(circle_nav.get_radius() + 0.5f); // at 100Hz makes it 50 cm every second
        } else if (g.rc_2.control_in < 0 && circle_nav.get_radius() >= 0.0f) {
        	circle_nav.set_radius(circle_nav.get_radius() - 0.5f); // at 100Hz makes it 50 cm every second
        }

        //read roll channel and use it to update circle rate
        // To-Do: make variable increase based on stick position
        if(g.rc_1.control_in > 0 && circle_nav.get_rate() < 90.0f) {
        	circle_nav.set_rate(circle_nav.get_rate() + 0.1f); // at 100Hz makes it 10 rad/s every second
        } else if (g.rc_1.control_in < 0 && circle_nav.get_rate() > -90.0f) {
        	circle_nav.set_rate(circle_nav.get_rate() - 0.1f); // at 100Hz makes it 10 rad/s every second
        }


    }

    // run circle controller
    circle_nav.update();

    // call attitude controller
    if (circle_pilot_yaw_override) {
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), target_yaw_rate);
    }else{
        attitude_control.angle_ef_roll_pitch_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), circle_nav.get_yaw(),true);
    }

    // run altitude controller
    if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
        // if sonar is ok, use surface tracking
        target_climb_rate = get_throttle_surface_tracking(target_climb_rate, pos_control.get_alt_target(), G_Dt);
    }
    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
    pos_control.update_z_controller();
}
