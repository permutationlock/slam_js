/*
 * slam.js
 * Author: Aven Bross (dabross@alaska.edu)
 * 
 * Particle filter occupancy grid SLAM with distributed particle maps.
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
	var _this = this;
	_this.x = x;
	_this.y = y;
	_this.angle = angle;
	
	_this.add = function(location) {
		_this.x += location.x;
		_this.y += location.y;
		_this.angle += location.angle;
	};
	
	_this.equals = function(location) {
		return (
				(_this.x == location.x) &&
				(_this.y == location.y) &&
				(_this.angle == location.angle)
			);
	};
	
	_this.distance = function(location) {
		return Math.sqrt(
				Math.pow(_this.x - location.x, 2) +
				Math.pow(_this.y - location.y, 2)
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
	var _this = this;
	_this.current = current_location;
	_this.old = old_location;
	
	_this.still = _this.current.equals(_this.old);
};

/*
 * odometry_motion_model_t
 * Takes variance parameters a1, a2, a3, and a4. Provides function to predict
 * new locations from a given location based on odometry controls.
 */
var odometry_motion_model_t = function(a1, a2, a3, a4) {
	var _this = this;
	_this.a1 = a1;
	_this.a2 = a2;
	_this.a3 = a3;
	_this.a4 = a4;
	
	_this.sample = function(control, location) {
		if(control.still()) return location;
		
		var dx_bar = control.current.x - control.last.x;
		var dy_bar = control.current.y - control.last.y;
		
		var delta_r1 = Math.atan2(dy_bar, dx_bar) - control.last.angle;
		var delta_trans = Math.sqrt(Math.pow(dx_bar, 2) + Math.pow(dy_bar, 2));
		var delta_r2 = control.current.angle - control.last.angle - delta_r1;
		
		var dhat_r1 = delta_r1 - sample_normal(
				_this.a1 * Math.pow(delta_r1, 2) +
				_this.a2 * Math.pow(delta_trans, 2)
			);
		var dhat_trans = delta_trans - sample_normal(
				_this.a3 * Math.pow(delta_trans, 2) +
				_this.a4 * Math.pow(delta_r1, 2) +
				_this.a4 * Math.pow(delta_r2, 2)
			);
		var dhat_r2 = delta_r2 - sample_normal(
				_this.a1 * Math.pow(delta_r2, 2) +
				_this.a2 * Math.pow(delta_trans, 2)
			);
		
		var initial = new locaton_t(location.x, location.y, location.angle);
		var motion = location_from_polar(dhat_r1, dhat_trans);
		var turn = new location_t(0.0, 0.0, dhat_r2);
		
		return initial.add(motion).add(turn);
	};
};

/*
 * ray_trace
 * Trace a rasterized line from start_location to end_location, calling
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
	var _this = this;
	_this.variance = variance;
	_this.max_ray = max_ray;
	_this.size = size;
	_this.samples = samples;
	_this.range_size = size / samples;
	_this.start_index = 0;
	_this.delta_rot = 2.0 * Math.PI / _this.size;
	
	_this.prob_ray = function(robot_location, hit_location, map_lookup) {
		var copy_loc = copy_location(robot_location);
		var ray = location_from_polar(_this.max_ray, hit_location.angle);
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
			
			return prob_normal(actual, expected, _this.variance);
		}
		
		return 1.0;
	};
	
	_this.prob = function(robot_location, measurement, map_lookup) {
		var q = 1.0;
		var rot = -1.0 * _this.delta_rot * _this.start_index;
		
		for(var i = 0; i < _this.size; i += range_size) {
			if(measurement[i] != 0.0) {
				var copy_loc = copy_location(robot_location);
				var ray = location_from_polar(measurement[i], rot);
				
				q *= prob_ray(robot_location, copy_loc.add(ray), map_lookup);
			}
			
			rot -= _this.delta_rot;
		}
	};
	
	_this.update = function(robot_location, measurement, map_update) {
		var rot = -1.0 * _this.delta_rot * _this.start_index;
		
		for(var i = 0; i < _this.size; i += range_size) {
			if(measurement[i] != 0.0) {
				var copy_loc = copy_location(robot_location);
				var ray = location_from_polar(measurement[i], rot);
				
				ray_trace(
						robot_location,
						copy_loc.add(ray),
						function(x, y, n) {
							if(n > 0) {
								map_update(false, x, y);
							}
							else {
								map_update(true, x, y);
							}
						}
					);
			}
			
			rot -= _this.delta_rot;
		}
	};
	
	_this.increment = function() {
		_this.start_index = (_this.start_index + 1) % range_size;
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
	var _this = this;
	_this.size = size;
	_this.n = 1.0 / size;
	_this.threshold = elimination_factor * _this.n;
	_this.prediction_model = prediction_model;
	_this.weight_model = weight_model;
	_this.weights = [];
	for(var i = 0; i < _this.size; ++i) {
		_this.weigths[i] = _this.n;
	}
	
	_this.predict = function(particles, control) {
		var new_particles = [];
		for(var i = 0; i < _this.size; ++i) {
			new_particlles[i] = _this.prediction_model(particles[i], control);
		}
		return new_particles;
	};
	
	_this.weight = function(particles, measurement) {
		var sum = 0.0;
		for(var i = 0; i < _this.size; ++i) {
			if(_this.weights[i] > _this.threshold) {
				_this.weights[i] *= _this.weight_model(particles[i], measurement);
			}
			else {
				_this.weights[i] = 0.0;
			}
			sum += _this.weights[i];
		}
		
		for(var i = 0; i < _this.size; ++i) {
			if(sum > 0.0000000001) {
				_this.weights[i] /= sum;
			}
			else {
				_this.weights[i] = _this.n;
			}
		}
	};
	
	_this.effective_sample_size = function() {
		var sum = 0.0;
		for(var i = 0; i < _this.size; ++i) {
			sum += Math.pow(_this.weights[i], 2);
		}
		
		return 1.0 / sum;
	};
	
	_this.resample = function(particles) {
		var new_particles = [];
		var r = Math.random(), c = _this.weights[0], i = 0;
		
		for(var m = 0; m < _this.size; ++m) {
			var u = r + m * _this.n;
			while(u > c) {
				++i;
				c += _this.weights[i];
			}
			new_particles[m] = particles[i];
		}
		
		for(var i = 0; i < _this.size; ++i) {
			_this.weights[i] = _this.n;
		}
		
		return new_particles;
	};
};

/*
 * dp_map_t
 * Simple two dimensional distributed particle map with binary occupancy values.
 */
var dp_map_t = function() {
	var _this = this;
	_this.map = [][][];
	
	_this.lookup_by_id(x, y, id) {
		if(typeof _this.map[x][y][id] !== "undefined") {
			return _this.map[x][y][id];
		}
		return -1;
	};
	
	_this.lookup(x, y, dp_node) {
		var temp = dp_node;
		do {
			var val = _this.lookup_by_id(x, y, temp.id);
			if(val == 1) {
				return true;
			}
			else if(val == 0) {
				return false;
			}
		} while((temp = temp.parent) != null);
		return false;
	};
	
	_this.update_by_id(value, x, y, id) {
		_this.map[x][y][id] = value;
	};
	
	_this.update(value, x, y, dp_node) {
		var temp = dp_node;
		do {
			if(_this.lookup_by_id(x, y, temp.id) != -1) {
				return false;
			}
		} while((temp = temp.parent) != null);
		_this.update_by_id(value, x, y, dp_node.id);
		return true;
	};
	
	_this.erase(x, y, id) {
		delete _this.map[x][y][id];
	};
	
	_this.rename(x, y, old_id, new_id) {
		_this.map[x][y][new_id] = _this.map[x][y][old_id];
		_this.erase(x, y, old_id);
	};
};

/*
 * dp_node_t
 * Node class to construct distributed particle trees.
 */
var dp_node_t = function(id, location, parent) {
	var _this = this;
	_this.id = id;
	_this.location_t = copy_location(location);
	if(parent != null) {
		parent.leaf = false;
		parent.children += 1;
	}
	_this.parent = parent;
	_this.leaf = false;
	_this.children = 0;
	_this.modified_cells = [];
	
	_this.add_cell = function(x, y) {
		_this.modified_cells.push({ "x" : x, "y" : y });
	};
	
	_this.trim = function(map) {
		if(_this.parent.id == 0) return;
		
		if(!_this.leaf && _this.children == 0) {
			_this.parent.children -= 1;
			for(var i = 0; i < _this.modified_cells.length; ++i) {
				map.erase(
						_this.modified_cells[i].x,
						_this.modified_cells[i].y,
						_this.id
					);
			}
			_this.parent.trim(map);
			_this.parent = null;
		}
		else if(_this.parent.children == 1 && _this.parent.id != 0) {
			for(var i = 0; i < _this.modified_cells.length; ++i) {
				map.rename(
						_this.modified_cells[i].x,
						_this.modified_cells[i].y,
						_this.id,
						_this.parent.id
					);
				_this.parent.modified_cells.push(_this.modified_cells[i]);
			}
			
			_this.modified_cells = _this.parent.modified_cells;
			_this.id = _this.parent.id;
			
			_this.parent = _this.parent.parent;
			_this.parent.trim(map);
		}
	};
};

/*
 * dp_slam_t
 * Distributed particle slam object. Tracks robot pose and map with particle
 * filter based on the given motion and measurement models.
 */
var dp_slam_t = function(size, motion_model, measurement_model, frac = 0.5) {
	var _this = this;
	_this.size = size;
	_this.resample_size = _this.size * frac;
	_this.next_id = 0;
	_this.root = new dp_node_t(next_id++, new location_t(0.0, 0.0, 0.0), null);
	_this.motion_model = motion_model;
	_this.measurement_model = measurement_model;
	_this.particles = [];
	_this.map = new dp_map_t;
	for(var i = 0; i < _this.size; ++i) {
		particles.push(new dp_node_t(
				next_id++,
				new location_t(0.0, 0.0, 0.0),
				root
			));
		
	}
	_this.particle_filter = new particle_filter_t(
			function(dp_node, control) {
				var location = _this.motion_model.sample(control, particle.location);
				return new dp_node_t(_this.next_id++, location, dp_node);
			},
			function(dp_node, measurement) {
				return _this.measurement_model.prob(
						dp_node.location,
						measurement,
						function(x, y) {
							return _this.map.lookup(x, y, dp_node);
						}
					);
			},
			_this.size
		);
	
	_this.update(control, measurement) {
		_this.particle_filter.predict(_this.particles, control);
		
		_this.particle_filter.weight(_this.particles, measurement);
		
		if(_this.particle_filter.effective_sample_size() < resample_size) {
			var new_particles = _this.particle_filter.resample(particles);
			
			for(var i = 0; i < _this.size; ++i) {
				new_particles[i].leaf = true;
			}
			
			for(var i = 0; i < _this.size; ++i) {
				_this.particles[i].trim();
			}
			
			_this.particles = new_particles;
		}
		
		for(var i = 0; i < _this.size; ++i) {
			var dp_node = _this.particles[i];
			_this.measurement_model.update(
					_this.particles.location,
					measurement,
					function(value, x, y) {
						if(_this.map.update(value, x, y, dp_node)) {
							dp_node.add_cell(x, y);
						}
					}
				);
		}
		
		_this.measurement_model.increment();
	};
};

