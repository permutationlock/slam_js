/*
 * slam.js
 * Author: Aven Bross (dabross@alaska.edu)
 * 
 * Particle filter occupancy grid SLAM with distributed particle maps and
 * people tracking with conditional particle filters.
 */

/*
 * sample_normal
 * Sample a random point according to a normal with the given parameters.
 */
function sample_normal(mean = 0.0, variance = 1.0) {
	var u = 1 - Math.random(); // Subtraction to flip [0, 1) to (0, 1].
	var v = 1 - Math.random();
	return mean + Math.sqrt( -2.0 * variance * Math.log( u ) )
		* Math.cos( 2.0 * Math.PI * v );
}

/*
 * prob_normal
 * Computes the probability density function of a normal distribution with the
 * given parameters at the given evaluation value.
 */
function prob_normal(value, mean = 0.0, variance = 1.0) {
	return (1.0 / Math.sqrt(2.0 * Math.PI * variance))
		* Math.exp(-0.5 * Math.pow(value - mean, 2) / variance);
}

/*
 * location_t
 * Stores coordinates and orientation of an object. Allows addition,
 * and comparison of locations.
 */
var location_t = function(x, y, angle) {
	this.x = x;
	this.y = y;
	this.angle = angle;
	
	this.add = function(location) {
		this.x += location.x;
		this.y += location.y;
		this.angle += location.angle;
	};
	
	this.equals = function(location) {
		return (
				(this.x == location.x) &&
				(this.y == location.y) &&
				(this.angle == location.angle)
			);
	};
	
	this.distance = function(location) {
		return Math.sqrt(
				Math.pow(this.x - location.x, 2) +
				Math.pow(this.y - location.y, 2)
			);
	};
};

/*
 * location_from_polar
 * Construct a location_t from polar coordinates.
 */
function location_from_polar(r, angle) {
	var x = Math.cos(angle) * r;
	var y = Math.sin(angle) * r;
	
	return new location_t(x, y, angle);
}

/*
 * control_t
 * Stores odometry information between two timesteps.
 */
var control_t = function(current_location, old_location) {
	this.current = current_location;
	this.old = old_location;
	
	this.still = this.current.equals(this.old);
};

/*
 * odometry_motion_model_t
 * Takes variance parameters a1, a2, a3, and a4. Provides function to predict
 * new locations from a given location based on odometry controls.
 */
var odometry_motion_model_t = function(a1, a2, a3, a4) {
	this.a1 = a1;
	this.a2 = a2;
	this.a3 = a3;
	this.a4 = a4;
	
	this.sample = function(control, location) {
		if(control.still()) return location;
		
		var dx_bar = control.current.x - control.last.x;
		var dy_bar = control.current.y - control.last.y;
		
		var delta_r1 = Math.atan2(dy_bar, dx_bar) - control.last.angle;
		var delta_trans = Math.sqrt(Math.pow(dx_bar, 2) + Math.pow(dy_bar, 2));
		var delta_r2 = control.current.angle - control.last.angle - delta_r1;
		
		var dhat_r1 = delta_r1 - sample_normal(
				this.a1 * Math.pow(delta_r1, 2) +
				this.a2 * Math.pow(delta_trans, 2)
			);
		var dhat_trans = delta_trans - sample_normal(
				this.a3 * Math.pow(delta_trans, 2) +
				this.a4 * Math.pow(delta_r1, 2) +
				this.a4 * Math.pow(delta_r2, 2)
			);
		var dhat_r2 = delta_r2 - sample_normal(
				this.a1 * Math.pow(delta_r2, 2) +
				this.a2 * Math.pow(delta_trans, 2)
			);
		
		var motion = location_from_polar(dhat_r1, dhat_trans);
		var turn = new location_t(0.0, 0.0, dhat_r2);
		
		return location.add(motion).add(turn);
	};
};

/*
 * ray_trace
 * Trace a discrete line from start_location to end_location, calling
 * evalute_cell on each cell. The evaluate_cell function must take x and y
 * coordinates, and n the number of cells remaining on the line.
 */
function ray_trace(start_location, end_location, evaluate_cell) {
	var x0 = start_location.x, y0 = stat_location.y;
	var x1 = end_location.x, y1 = end_location.y;
	var dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
	
	var x = Math.floor(x0), y = Math.floor(y0), n = 1;
	var x_inc, y_inc, error;
	
	if(dx == 0) {
		x_inc = 0;
		error = 100000;
	}
	else if(x1 > x0) {
		x_inc = 1;
		n += Math.floor(x1) - x;
		error = (Math.floor(x0) + 1.0 - x0) * dy;
	}
	else {
		x_inc = -1;
		n += x - Math.floor(x1);
		error = (x0 - Math.floor(x0)) * dy;
	}
	
	if(dy == 0) {
		y_inc = 0;
		error = -100000;
	}
	else if(y1 > y0) {
		y_inc = 1;
		n += Math.floor(y1) - y;
		error = (Math.floor(y0) + 1.0 - y0) * dx;
	}
	else {
		y_inc = -1;
		n += y - Math.floor(y1);
		error -= (y0 - Math.floor(y0)) * dx;
	}
	
	for(; n > 0; --n) {
		evaluate_cell(x, y, n - 1);
	}
};

/*
 * beam_measurement_model_t
 * Takes range sensor variance. Provides functions to compute the probability
 * of measurement vectors and individual measurements for a given map and
 * robot location.
 */
var beam_measurement_model_t = function(variance, samples) {
	this.variance = variance;
	
	this.prob_ray = function(robot_location, hit_location, map) {
		
	};
};

