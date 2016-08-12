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
 * copy_location
 * Construct a location_t from another.
 */
function copy_location(location) {
	return new location_t(location.x, location.y, location.angle);
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
		
		var initial = new locaton_t(location.x, location.y, location.angle);
		var motion = location_from_polar(dhat_r1, dhat_trans);
		var turn = new location_t(0.0, 0.0, dhat_r2);
		
		return initial.add(motion).add(turn);
	};
};

/*
 * ray_trace
 * Trace a discrete line from start_location to end_location, calling
 * evalute_cell on each cell. The evaluate_cell function must take x and y
 * coordinates, and n the number of cells remaining on the line. We also
 * compute a distance_per_cell value to approximately handle the fact that
 * diagonal traces will hit more cells than horizontal traces.
 */
function ray_trace(start_location, end_location, evaluate_cell) {
	var x0 = start_location.x, y0 = stat_location.y;
	var x1 = end_location.x, y1 = end_location.y;
	var dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
	
	var x = Math.floor(x0), y = Math.floor(y0), n = 1;
	var x_inc, y_inc, error = 0.0;
	
	if(dx == 0) {
		x_inc = 0;
		error = 100000;
	}
	else if(x1 > x0) {
		x_inc = 1;
		n += Math.floor(x1) - x;
		error += (Math.floor(x0) + 1.0 - x0) * dy;
	}
	else {
		x_inc = -1;
		n += x - Math.floor(x1);
		error += (x0 - Math.floor(x0)) * dy;
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
		if(evaluate_cell(x, y, n - 1)) return;
		if(error > 0) {
			y += y_inc;
			error -= dx;
		}
		else {
			x += x_inc;
			error += dy;
		}
	}
};

/*
 * beam_measurement_model_t
 * Takes range sensor variance. Provides functions to compute the probability
 * of measurement vectors and individual measurements for a given map and
 * robot location.
 */
var beam_measurement_model_t = function(variance, max_ray, samples, size) {
	this.variance = variance;
	this.max_ray = max_ray;
	this.size = size;
	this.samples = samples;
	this.range_size = size / samples;
	this.start_index = 0;
	this.delta_rot = 2.0 * Math.PI / this.size;
	
	this.prob_ray = function(robot_location, hit_location, map_lookup) {
		var copy_loc = copy_location(robot_location);
		var ray = location_from_polar(this.max_ray, hit_location.angle);
		var exp_hit = null;
		
		ray_trace(
				robot_location,
				copy_loc.add(ray),
				function(x, y, n) {
					if(map_lookup(x, y)) {
						exp_hit = new robot_location_t(x + 0.5, y + 0.5, 0.0);
						
						return true;
					}
					
					return false;
				}
			);
		
		if(end_point != null) {
			var actual = robot_location.distance(hit_location);
			var expected = robot_location.distance(exp_hit);
			
			return prob_normal(actual, expected, this.variance);
		}
		
		return 1.0;
	};
	
	this.prob_measurement = function(robot_location, measurement, map_lookup) {
		var q = 1.0;
		var rot = -1.0 * this.delta_rot * this.start_index;
		
		for(var i = 0; i < this.size; i += range_size) {
			if(measurement[i] != 0.0) {
				var copy_loc = copy_location(robot_location);
				var ray = location_from_polar(measurement[i], rot);
				
				q *= prob_ray(robot_location, copy_loc.add(ray), 
			}
			
			rot -= this.delta_rot;
		}
		
		this.start_index = (this.start_index + 1) % range_size;
	};
};

/*
 * particle_filter_t
 * A simple particle filter to estimate a posterior distribution over a finite
 * number of sample points. Call predict and weight each timestep to estimate
 * the posterior distribution with controls and measurements. Call resample to
 * eliminate low probability particles when needed to produce better estimates
 * with lower particle counts.
 */
var particle_filter_t = function(prediction_model, weight_model, size,
	elimination_factor = 0.01)
{
	this.size = size;
	this.n = 1.0 / size;
	this.threshold = elimination_factor * this.n;
	this.prediction_model = prediction_model;
	this.weight_model = weight_model;
	this.weights = [];
	for(var i = 0; i < this.size; ++i) {
		this.weigths[i] = this.n;
	}
	
	this.predict = function(particles, control) {
		var new_particles = [];
		for(var i = 0; i < this.size; ++i) {
			new_particlles[i] = this.prediction_model(particles[i], control);
		}
		return new_particles;
	};
	
	this.weight = function(particles, measurement) {
		var sum = 0.0;
		for(var i = 0; i < this.size; ++i) {
			if(this.weights[i] > this.threshold) {
				this.weights[i] *= this.weight_model(particles[i], measurement);
			}
			else {
				this.weights[i] = 0.0;
			}
			sum += this.weights[i];
		}
		
		for(var i = 0; i < this.size; ++i) {
			if(sum > 0.0000000001) {
				this.weights[i] /= sum;
			}
			else {
				this.weights[i] = this.n;
			}
		}
	};
	
	this.effective_sample_size = function() {
		var sum = 0.0;
		for(var i = 0; i < this.size; ++i) {
			sum += Math.pow(this.weights[i], 2);
		}
		
		return 1.0 / sum;
	};
	
	this.resample = function(particles) {
		var new_particles = [];
		var r = Math.random(), c = this.weights[0], i = 0;
		
		for(var m = 0; m < this.size; ++m) {
			var u = r + m * this.n;
			while(u > c) {
				++i;
				c += this.weights[i];
			}
			new_particles[m] = particles[i];
		}
		
		for(var i = 0; i < this.size; ++i) {
			this.weights[i] = this.n;
		}
		
		return new_particles;
	};
};

/*
 * dp_map
 * Simple two dimensional map with binary occupancy values.
 */
var dp_map = function(size) {
	this.size = size;
	this.map = [][]{};
	
	this.lookup_by_id(x, y, id) {
		if(this.map[x][y].id != null) {
			return this.map[x][y][id];
		}
		return -1;
	};
	
	this.lookup(x, y, dp_node) {
		var temp = dp_node;
		do {
			var val = lookup_by_id(x, y, temp.id);
			if(val == 1) {
				return true;
			}
			else if(val == 0) {
				return false;
			}
		} while((temp = temp.parent) != null);
		return false;
	};
	
	this.update_by_id(value, x, y, id) {
		this.map[x][y][id] = value;
	};
	
	this.update(value, x, y, dp_node) {
		var temp = dp_node;
		do {
			if(lookup_by_id(x, y, temp.id) != -1) {
				return;
			}
		} while((temp = temp.parent) != null);
		update_by_id(value, x, y, dp_node.id);
	};
};

