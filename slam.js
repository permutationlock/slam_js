/*
 * slam.js
 * Author: Aven Bross (dabross@alaska.edu)
 * 
 * Occupancy grid SLAM with distributed particle maps and people tracking with
 * conditional particle filters.
 */

function sample_normal(mean = 0.0, variance = 1.0) {
	var u = 1 - Math.random(); // Subtraction to flip [0, 1) to (0, 1].
    var v = 1 - Math.random();
    return mean + Math.sqrt( -2.0 * variance * Math.log( u ) ) * Math.cos( 2.0 * Math.PI * v );
}

var location_t = function(
