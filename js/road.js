function road(roadID, roadLen, nLanes, densInitPerLane, speedInit, truckFracInit, isRing) {
	this.roadID = roadID;
	this.swarms = [];
	this.ambulances = [];
	this.roadLen = roadLen;
	this.nLanes = nLanes;
	this.nveh = Math.floor(this.nLanes * this.roadLen * densInitPerLane);
	this.isRing = isRing;
	this.inVehBuffer = 0; // number of waiting vehicles; if>=1, updateBCup called
	this.iOffset = 0; // set by getTargetNeighbourhood: first veh in defined region
	this.duTactical = -1e-6; // if duAntic>0 activate tactical changes for mandat. LC

	this.MOBIL_bSafeMandat = 6; // mandat LC and merging for v=v0
	this.MOBIL_bSafeMax = 17; //!!! mandat LC and merging for v=0


	this.LCModelMandatoryRight = new MOBIL(this.MOBIL_bSafeMandat,
		this.MOBIL_bSafeMax,
		0, 0.5 * this.MOBIL_bSafeMax);
	this.LCModelMandatoryLeft = new MOBIL(this.MOBIL_bSafeMandat,
		this.MOBIL_bSafeMandat,
		0, -0.5 * this.MOBIL_bSafeMax);

	this.draw_scaleOld = 0;
	this.draw_nSegm = 100;
	this.draw_curvMax = 0.01; // maximum assmued curvature

	this.draw_x = [];  // arrays defined in the draw(..) method
	this.draw_y = [];
	this.draw_phi = [];
	this.draw_cosphi = [];
	this.draw_sinphi = [];


	this.veh = [];
	for (var i = 0; i < this.nveh; i++) {


		var lane = i % this.nLanes; // left: 0; right: nLanes-1
		var truckFracRight = Math.min(this.nLanes * truckFracInit, 1);
		var truckFracRest = (this.nLanes * truckFracInit > 1)
			? ((this.nLanes * truckFracInit - 1) / (this.nLanes - 1)) : 0;
		var truckFrac = (lane == this.nLanes - 1) ? truckFracRight : truckFracRest;
		var r = Math.random();
		var vehType = (Math.random() < truckFrac) ? "truck" : "car";
		var vehLength = (vehType == "car") ? car_length : truck_length;
		var vehWidth = (vehType == "car") ? car_width : truck_width;

		this.veh[i] = new Vehicle(vehLength, vehWidth,
			(this.nveh - i - 1) * this.roadLen / (this.nveh + 1),
			i % this.nLanes, 0.8 * speedInit, vehType);

		this.veh[i].longModel = (vehType == "car")
			? longModelCar : longModelTruck;
		this.veh[i].LCModel = (vehType == "car")
			? LCModelCar : LCModelTruck;
	}

}



//#####################################################
// set vehicles in range to new CF models
// (useful for modeling flow-conserving bottlenecks)
//#####################################################



//#####################################################
// set vehicles in range to mandatory LC 
// (useful for non-routing related mandatory LC onramps (no offramps), e.g.
// onramps or before lane closings
// see also updateModelsOfAllVehicles
//#####################################################

road.prototype.setLCMandatory = function (umin, umax, toRight) {
	for (var i = 0; i < this.veh.length; i++) {
		var u = this.veh[i].u;
		if ((u > umin) && (u < umax)) {
			this.veh[i].mandatoryLCahead = true;
			this.veh[i].toRight = toRight;
			this.veh[i].LCModel = (toRight)
				? this.LCModelMandatoryRight : this.LCModelMandatoryLeft;
		}
	}
}

//#####################################################
// sort vehicles into descending arc-length positions u 
//#####################################################

road.prototype.sortVehicles = function () {
	if (this.veh.length > 2) {
		this.veh.sort(function (a, b) {
			return b.u - a.u;
		})
	};
}



road.prototype.update_iLead = function (i) {
	var n = this.nveh;
	this.veh[i].iLeadOld = this.veh[i].iLead;
	var iLead = (i == 0) ? n - 1 : i - 1;  // also for non periodic BC
	success = (this.veh[iLead].lane == this.veh[i].lane);
	while (!success) {
		iLead = (iLead == 0) ? n - 1 : iLead - 1;
		success = ((i == iLead) || (this.veh[iLead].lane == this.veh[i].lane));
	}
	this.veh[i].iLead = iLead;
}

// get/update follower

road.prototype.update_iLag = function (i) {
	var n = this.nveh;
	this.veh[i].iLagOld = this.veh[i].iLag;
	var iLag = (i == n - 1) ? 0 : i + 1;
	success = (this.veh[iLag].lane == this.veh[i].lane);
	while (!success) {
		iLag = (iLag == n - 1) ? 0 : iLag + 1;
		success = ((i == iLag) || (this.veh[iLag].lane == this.veh[i].lane));
	}
	this.veh[i].iLag = iLag;
}


// get leader to the right

road.prototype.update_iLeadRight = function (i) {
	var n = this.nveh;
	this.veh[i].iLeadRightOld = this.veh[i].iLeadRight;
	var iLeadRight;
	if (this.veh[i].lane < this.nLanes - 1) {
		iLeadRight = (i == 0) ? n - 1 : i - 1;
		success = ((i == iLeadRight) || (this.veh[iLeadRight].lane == this.veh[i].lane + 1));
		while (!success) {
			iLeadRight = (iLeadRight == 0) ? n - 1 : iLeadRight - 1;
			success = ((i == iLeadRight) || (this.veh[iLeadRight].lane == this.veh[i].lane + 1));
		}
	}
	else { iLeadRight = -10; }
	this.veh[i].iLeadRight = iLeadRight;
}

// get follower to the right

road.prototype.update_iLagRight = function (i) {
	var n = this.nveh;
	this.veh[i].iLagRightOld = this.veh[i].iLagRight;
	var iLagRight;
	if (this.veh[i].lane < this.nLanes - 1) {
		iLagRight = (i == n - 1) ? 0 : i + 1;
		success = ((i == iLagRight) || (this.veh[iLagRight].lane == this.veh[i].lane + 1));
		while (!success) {
			iLagRight = (iLagRight == n - 1) ? 0 : iLagRight + 1;
			success = ((i == iLagRight) || (this.veh[iLagRight].lane == this.veh[i].lane + 1));
		}
	}
	else { iLagRight = -10; }
	this.veh[i].iLagRight = iLagRight;
}

// get leader to the left

road.prototype.update_iLeadLeft = function (i) {
	var n = this.nveh;
	this.veh[i].iLeadLeftOld = this.veh[i].iLeadLeft;

	var iLeadLeft;
	if (this.veh[i].lane > 0) {
		iLeadLeft = (i == 0) ? n - 1 : i - 1;
		success = ((i == iLeadLeft) || (this.veh[iLeadLeft].lane == this.veh[i].lane - 1));
		while (!success) {
			iLeadLeft = (iLeadLeft == 0) ? n - 1 : iLeadLeft - 1;
			success = ((i == iLeadLeft) || (this.veh[iLeadLeft].lane == this.veh[i].lane - 1));
		}
	}
	else { iLeadLeft = -10; }
	this.veh[i].iLeadLeft = iLeadLeft;
}

// get follower to the left

road.prototype.update_iLagLeft = function (i) {
	var n = this.nveh;
	var iLagLeft;
	this.veh[i].iLagLeftOld = this.veh[i].iLagLeft;

	if (this.veh[i].lane > 0) {
		iLagLeft = (i == n - 1) ? 0 : i + 1;
		success = ((i == iLagLeft) || (this.veh[iLagLeft].lane == this.veh[i].lane - 1));
		while (!success) {
			iLagLeft = (iLagLeft == n - 1) ? 0 : iLagLeft + 1;
			success = ((i == iLagLeft) || (this.veh[iLagLeft].lane == this.veh[i].lane - 1));
		}
	}
	else { iLagLeft = -10; }
	this.veh[i].iLagLeft = iLagLeft;
}


//#####################################################
// get/update environment iLead, iLag, iLeadLeft,... for all vehicles
//#####################################################

road.prototype.updateEnvironment = function () {
	for (var i = 0; i < this.nveh; i++) {
		// get leader
		this.update_iLead(i);
		
		// get follower
		this.update_iLag(i);
		
		// get leader to the right (higher lane index)
		this.update_iLeadRight(i);
		
		// get follower to the right
		this.update_iLagRight(i);
		
		// get leader to the left (lower lane index)
		this.update_iLeadLeft(i);
		
		// get follower to the left
		this.update_iLagLeft(i);
		this.formSwarms()
	}
}




//######################################################################
// main calculation of accelerations 
//######################################################################


road.prototype.calcAccelerations = function () {
    let ambulanceLanes = new Set();

    // 🚑 Step 1: Identify lanes with ambulances
    for (let j = 0; j < this.nveh; j++) {
        if (this.veh[j].isAmbulance) {
            ambulanceLanes.add(this.veh[j].lane);
        }
    }

    // 🚀 Step 2: Loop through all vehicles
    for (let i = 0; i < this.nveh; i++) {
        let veh = this.veh[i];
        let iLead = veh.iLead;

        // 🚗 Calculate gap to leader
        let s = (iLead >= 0) ? this.veh[iLead].u - this.veh[iLead].length - veh.u : 10000;
        if (iLead >= i) { 
            s = 10000; // Free road ahead if no leader
        }

        // 🚑 Step 3: Check if vehicle is in an ambulance lane
        if (ambulanceLanes.has(veh.lane)) {
            let ambulanceBehind = false;
            let ambulanceDistance = Infinity;

            // Find nearest ambulance in the same lane
            for (let j = 0; j < this.nveh; j++) {
                let otherVeh = this.veh[j];

                if (otherVeh.isAmbulance && otherVeh.lane === veh.lane) {
                    let distance = veh.u - otherVeh.u; // Distance from ambulance
                    if (distance > 0) {
                        ambulanceBehind = true;
                        ambulanceDistance = Math.min(ambulanceDistance, distance);
                    }
                }
            }

            // 🚀 Step 4: Accelerate all vehicles in the lane
            if (ambulanceBehind) {
                let baseAcc = veh.longModel.calcAcc(s, veh.speed, this.veh[iLead]?.speed || 0);
                let extraAcc = Math.max(0, (100 - ambulanceDistance) * 0.05); // Gradual boost

                // 🛑 Prevent stacking
                if (s < 10) {  
                    extraAcc = -0.5; // Reduce acceleration if too close
                }

                veh.acc = Math.min(baseAcc + extraAcc, 2); // Cap acceleration at 2 m/s²
                // console.log(`🚑 Vehicle at u=${veh.u} accelerates! (acc=${veh.acc}, gap=${s})`);
            } else {
                veh.acc = veh.longModel.calcAcc(s, veh.speed, this.veh[iLead]?.speed || 0);
            }
        } else {
            // Normal acceleration if no ambulance
            veh.acc = veh.longModel.calcAcc(s, veh.speed, this.veh[iLead]?.speed || 0);
        }
    }
};

road.prototype.updateSpeedPositions = function () {
    for (let i = 0; i < this.nveh; i++) {
        let veh = this.veh[i];

        // 🚗 **Update position with smooth acceleration**
        veh.u += Math.max(0, veh.speed * dt + 0.5 * veh.acc * dt * dt);

        // 🚦 **Update speed**
        veh.speed += veh.acc * dt;
        if (veh.speed < 0) { veh.speed = 0; } // No negative speeds

        // 📏 **Ensure smooth lane changing behavior**
        veh.v = get_v(veh.dt_lastLC, dt_LC, veh.laneOld, veh.lane);
    }

    this.updateOrientation(); // Update drawing orientation
    this.sortVehicles(); // Maintain correct vehicle order
    this.updateEnvironment(); // Update traffic environment
};


road.prototype.changeLanes = function () {
	this.doChangesInDirection(1); // changes to right 
	this.doChangesInDirection(0); // changes to left 
}



road.prototype.doChangesInDirection = function (toRight) {
    var log = false;
    var waitTime = 2 * dt_LC;

    if (log) { console.log(`🚗 Lane Change Attempt: ${toRight ? "Right" : "Left"}`); }

    // 🚑 Step 1: Identify current ambulance lanes dynamically
    let ambulanceLanes = new Set();
    for (let j = 0; j < this.nveh; j++) {
        if (this.veh[j].isAmbulance) {
            ambulanceLanes.add(this.veh[j].lane);
        }
    }

    // 🚗 Step 2: Loop through all vehicles
    for (var i = 0; i < this.nveh; i++) {
        var veh = this.veh[i];
        var newLane = toRight ? veh.lane + 1 : veh.lane - 1;

        // 🚑 Step 3: Prevent switching into an active ambulance lane
        if (ambulanceLanes.has(newLane)) {
            continue;
        }

        // 🛑 Step 4: Check if lane change is possible
        if ((newLane >= 0) && (newLane < this.nLanes) && (veh.dt_lastLC > waitTime)) {
            var iLeadNew = toRight ? veh.iLeadRight : veh.iLeadLeft;
            var iLagNew = toRight ? veh.iLagRight : veh.iLagLeft;

            // 🚥 Step 5: Ensure the new lane's leader and follower are also stable
            if ((this.veh[iLeadNew]?.dt_lastLC > waitTime) &&
                (this.veh[iLagNew]?.dt_lastLC > waitTime)) {

                var speed = veh.speed;
                var sNew = this.veh[iLeadNew]?.u - this.veh[iLeadNew]?.length - veh.u || 10000;
                var speedLeadNew = this.veh[iLeadNew]?.speed || 0;
                var accNew = veh.longModel.calcAcc(sNew, speed, speedLeadNew);

                var sLagNew = veh.u - veh.length - (this.veh[iLagNew]?.u || -10000);
                var speedLagNew = this.veh[iLagNew]?.speed || 0;
                var accLagNew = this.veh[iLagNew]?.longModel.calcAcc(sLagNew, speedLagNew, speed);

                // 📌 Step 6: Perform MOBIL safety check before changing lanes
                var vrel = speed / veh.longModel.v0;
                var MOBILOK = veh.LCModel.realizeLaneChange(vrel, veh.acc, accNew, accLagNew, toRight, false);

                if ((veh.type !== "obstacle") && (sNew > 0) && (sLagNew > 0) && MOBILOK) {
                    // 🚀 Step 7: Execute Lane Change
                    veh.dt_lastLC = 0;  // Reset lane change timer
                    if (iLagNew >= 0) this.veh[iLagNew].dt_lastPassiveLC = 0;
                    if (iLeadNew >= 0) this.veh[iLeadNew].dt_lastPassiveLC = 0;

                    veh.laneOld = veh.lane;
                    veh.lane = newLane;
                    veh.acc = accNew;
                    if (iLagNew >= 0) this.veh[iLagNew].acc = accLagNew;

                    if (log) {
                        console.log(`🚗 Vehicle ${i} moved from lane ${veh.laneOld} to ${veh.lane}`);
                    }

                    // 🔄 Step 8: Update Environment
                    this.updateEnvironment();
                }
            }
        }
    }
};



road.prototype.mergeDiverge = function (newRoad, offset, uStart, uEnd, isMerge, toRight) {

	var log = false;
	if (log) { console.log("\n\nitime=" + itime + ": in road.mergeDiverge"); }

	// (1) get neighbourhood

	var uNewStart = uStart + offset;
	var uNewEnd = uEnd + offset;
	var padding = 50; // additional visibility  on target road before/after
	var originLane = (toRight) ? this.nLanes - 1 : 0;
	var targetLane = (toRight) ? 0 : newRoad.nLanes - 1;

	// getTargetNeighbourhood also sets this.iOffset, newRoad.iOffset
	var originVehicles = this.getTargetNeighbourhood(uStart, uEnd, originLane);

	var targetVehicles = newRoad.getTargetNeighbourhood(
		uNewStart - padding, uNewEnd + padding, targetLane);

	var iMerge = 0; // candidate 
	var uTarget; // arc-length coordinate of the successfully changing veh(if any)

	var success = ((targetVehicles.length == 0) && (originVehicles.length > 0)
		&& (originVehicles[0].type != "obstacle")
		&& (originVehicles[0].mandatoryLCahead));
	if (success) { iMerge = 0; uTarget = originVehicles[0].u + offset; }

	else if (originVehicles.length > 0) {  // or >1 necessary? !!
		var duLeader = 1000; // initially big distances w/o interaction
		var duFollower = -1000;
		var leaderNew = new Vehicle(0, 0, uNewStart + 10000, targetLane, 0, "car");
		var followerNew = new Vehicle(0, 0, uNewStart - 10000, targetLane, 0, "car");
		if (log) { console.log("entering origVeh loop"); }
		for (var i = 0; (i < originVehicles.length) && (!success); i++) {// merging veh loop
			if ((originVehicles[i].type != "obstacle") && (originVehicles[i].mandatoryLCahead)) {
				uTarget = originVehicles[i].u + offset;
				if (log) { console.log(" i=" + i); }
				for (var j = 0; j < targetVehicles.length; j++) {
					var du = targetVehicles[j].u - uTarget;
					if ((du > 0) && (du < duLeader)) {
						duLeader = du; 
						leaderNew = targetVehicles[j];
					}
					if ((du < 0) && (du > duFollower)) {
						duFollower = du; 
						followerNew = targetVehicles[j];
					}
					if (log) {
						console.log("  du=" + du + " duLeader=" + duLeader
							+ " duFollower=" + duFollower);

					}

				}

				// get input variables for MOBIL

				var sNew = duLeader - leaderNew.length;
				var sLagNew = -duFollower - originVehicles[i].length;
				var speedLeadNew = leaderNew.speed;
				var speedLagNew = followerNew.speed;
				var speed = originVehicles[i].speed;

				var bSafeMergeMin = this.MOBIL_bSafeMandat;
				var bSafeMergeMax = this.MOBIL_bSafeMax;
				var bBiasMerge = (toRight) ? 0.5 * bSafeMergeMax
					: -0.5 * bSafeMergeMax; // strong urge to change
				var longModel = originVehicles[i].longModel;

				//!!! this alt: LCModel with locally defined bSafe params 6 and 17
				var LCModel = new MOBIL(bSafeMergeMin, bSafeMergeMax, 0, bBiasMerge);

				//!!! this alt: LCModel* overwritten from top-level routines! bSafe=42
				//var LCModel=(toRight) ? this.LCModelMandatoryRight 
				// : this.LCModelMandatoryLeft; 

				var vrel = originVehicles[i].speed / originVehicles[i].longModel.v0;
				var acc = originVehicles[i].acc;
				var accNew = longModel.calcAcc(sNew, speed, speedLeadNew);
				var accLagNew = longModel.calcAcc(sLagNew, speedLagNew, speed);

				// lane changing to merge on new road (regular LC above)
				var MOBILOK = LCModel.realizeLaneChange(vrel, acc, accNew, accLagNew, toRight, false);

				success = MOBILOK && (originVehicles[i].type != "obstacle")
					&& (sNew > 0) && (sLagNew > 0)
					&& (originVehicles[i].mandatoryLCahead);

				if (log && (this.roadID == 2)) {
					console.log("in road.mergeDiverge: roadID=" + this.roadID
						+ " LCModel.bSafeMax=" + LCModel.bSafeMax);
				}
				if (success) { iMerge = i; }

			} // !obstacle

		}// merging veh loop
	}// else branch (there are target vehicles)


	if (success) {// do the actual merging 

		//originVehicles[iMerge]=veh[iMerge+this.iOffset] 

		var iOrig = iMerge + this.iOffset;
	

		var changingVeh = this.veh[iOrig]; //originVehicles[iMerge];
		var vOld = (toRight) ? targetLane - 1 : targetLane + 1; // rel. to NEW road
		changingVeh.u += offset;
		changingVeh.lane = targetLane;
		changingVeh.laneOld = vOld; // following for  drawing purposes
		changingVeh.v = vOld;  // real lane position (graphical)

		changingVeh.dt_lastLC = 0;             // just changed
		changingVeh.mandatoryLCahead = false; // reset mandatory LC behaviour

		//!!! get index of this.veh and splice this; otherwise probably no effect 
		//####################################################################
		this.veh.splice(iOrig, 1);// removes chg veh from orig.
		newRoad.veh.push(changingVeh); // appends changingVeh at last pos;
		//####################################################################

		this.nveh = this.veh.length; // !! updates array lengths
		newRoad.nveh = newRoad.veh.length;
		newRoad.sortVehicles();       // move the mergingVeh at correct position
		newRoad.updateEnvironment(); // and provide updated neighbors

	}// end do the actual merging

}// end mergeDiverge




//######################################################################
// get heading (relative to road)
//######################################################################

road.prototype.updateOrientation = function () {
	for (var i = 0; i < this.nveh; i++) {
		this.veh[i].dvdu = get_dvdu(this.veh[i].dt_lastLC, dt_LC, // get_dvdu from paths.js
			this.veh[i].laneOld,
			this.veh[i].lane, this.veh[i].speed);
	}
}


//######################################################################
// update truck percentage by changing vehicle type of existing vehs
// do not correct if minor mismatch 
// since this can happen due to inflow/outflow
// open roads: mismatchTolerated about 0.2; ring: mismatchTolerated=0
//######################################################################

road.prototype.updateTruckFrac = function (truckFrac, mismatchTolerated) {
	if (this.veh.length > 0) {
		this.updateEnvironment(); // needs veh[i].iLag etc, so actual environment needed
		var n = this.veh.length;
		var nTruckDesired = Math.floor(n * truckFrac);
		var nTruck = 0;
		for (var i = 0; i < n; i++) {
			if (this.veh[i].type == "truck") { nTruck++; }
		}
		var truckFracReal = nTruck / n;  // integer division results generally in double: OK!

		// action if truck frac not as wanted; 
		// correct by one veh transformation per timestep

		if (Math.abs(truckFracReal - truckFrac) > mismatchTolerated) {
			var truckFracTooLow = (nTruckDesired > nTruck);
			var newType = (truckFracTooLow) ? "truck" : "car";
			var newLength = (truckFracTooLow) ? truck_length : car_length;
			var newWidth = (truckFracTooLow) ? truck_width : car_width;
			var newLongModel = (truckFracTooLow) ? longModelTruck : longModelCar;
			var diffSpace = ((truckFracTooLow) ? -1 : 1) * (truck_length - car_length);
			var success = 0; // false at beginning

			// find the candidate vehicle (truck or car) with the largest lag gap

			var candidateType = (truckFracTooLow) ? "car" : "truck";
			var k = 0;  // considered veh index

			if (truckFracTooLow) {// change cars->trucks on the right lane if possible
				var maxSpace = 0;
				for (var lane = this.nLanes - 1; lane >= 0; lane--) {
					if (!success) {
						for (var i = 0; i < n; i++) {
							if ((this.veh[i].lane == lane) && (this.veh[i].type == candidateType)) {
								var iLag = this.veh[i].iLag;
								var s = this.veh[i].u - this.veh[iLag].u - this.veh[i].length;
								if (iLag < i) { s += this.roadLen; }//periodic BC (OK for open BC as well)
								if (s > maxSpace) { k = i; maxSpace = s; }
								success = (maxSpace > diffSpace);
							}
						}
					}
				}
			}

			else { // change trucks->cars: transform truck with smallest space 
				var minSpace = 10000;
				for (var i = 0; i < n; i++) {
					if (this.veh[i].type == candidateType) {
						success = 1; // always true for trucks->cars if there is a truck
						var iLag = this.veh[i].iLag;
						var s = this.veh[i].u - this.veh[iLag].u - this.veh[i].length;
						if ((iLag < i) && (s < 0)) { s += this.roadLen; }//periodic BC (OK for open BC as well)
						if (s < minSpace) { k = i; minSpace = s; }
					}
				}
			}

			// actually do the transformation if no collision entails by it

			// console.log("in updateTruckFrac: nTruck="+nTruck+" nTruckDesired="+nTruckDesired+" k="+k+" maxSpace="+maxSpace+" candidateType=" +candidateType+" newType="+newType);

			if (success) {
				this.veh[k].type = newType;
				this.veh[k].length = newLength;
				this.veh[k].width = newWidth;
				this.veh[k].longModel = newLongModel;
			}
		}
	}
}





//######################################################################
// downstream BC: drop at most one vehicle at a time (no action needed if isRing)
//######################################################################

road.prototype.updateBCdown = function () {
	var nvehOld = this.nveh;
	if ((!this.isRing) && (this.veh.length > 0)) {
		if (this.veh[0].u > this.roadLen) {
			//console.log("road.updateBCdown: nveh="+this.nveh+" removing one vehicle);
			this.veh.splice(0, 1);
			this.nveh--;
		}
		if (this.nveh < nvehOld) { this.updateEnvironment(); }
	}
}

//######################################################################
// upstream BC: insert vehicles at total flow Qin (only applicable if !isRing)
// route is optional parameter (default: route=[])
//######################################################################

road.prototype.updateBCup = function (Qin, dt, route) {

	this.route = (typeof route === 'undefined') ? [0] : route; // handle opt. args
	var smin = 15; // only inflow if largest gap is at least smin
	var success = 0; // false initially
	if (!this.isRing) {
		this.inVehBuffer += Qin * dt;
	}
	if (this.inVehBuffer >= 1) {
		// get new vehicle characteristics
		var vehType = (Math.random() < truckFrac) ? "truck" : "car";
		var vehLength = (vehType == "car") ? car_length : truck_length;
		var vehWidth = (vehType == "car") ? car_width : truck_width;
		var space = 0; // available bumper-to-bumper space gap

		// try to set trucks at the right lane

		var lane = this.nLanes - 1; // start with right lane
		if (this.nveh == 0) { success = true; space = this.roadLen; }

		else if (vehType == "truck") {
			var iLead = this.nveh - 1;
			while ((iLead > 0) && (this.veh[iLead].lane != lane)) { iLead--; }
			space = this.veh[iLead].u - this.veh[iLead].length;
			success = (iLead < 0) || (space > smin);
		}

		// if road not empty or a truck could not be placed on the right lane 
		// try, as well as for cars, if there is any lane with enough space

		if (!success) {
			var spaceMax = 0;
			for (var candLane = this.nLanes - 1; candLane >= 0; candLane--) {
				var iLead = this.nveh - 1;
				while ((iLead >= 0) && (this.veh[iLead].lane != candLane)) { iLead--; }
				space = (iLead >= 0) // "minus candLine" implements right-driving 
					? this.veh[iLead].u - this.veh[iLead].length : this.roadLen + candLane;
				if (space > spaceMax) {
					lane = candLane;
					spaceMax = space;
				}
			}
			success = (space >= smin);
		}

		// actually insert new vehicle

		if (success) {
			var longModelNew = (vehType == "car") ? longModelCar : longModelTruck;
			var uNew = 0;
			var speedNew = Math.min(longModelNew.v0, longModelNew.speedlimit, space / longModelNew.T);
			var vehNew = new Vehicle(vehLength, vehWidth, uNew, lane, speedNew, vehType);
			vehNew.longModel = longModelNew;
			vehNew.route = this.route;

			this.veh.push(vehNew); // add vehicle after pos nveh-1
			this.inVehBuffer -= 1;
			this.nveh++;
		}
	}

}

//######################################################################
// get target vehicle neighbourhood/context for merging of other roads
// returns targetVehicles, an array of all vehicles on the target lane 
// inside the arclength range [umin, umax].
// Also sets iOffset, the first vehicle (smallest i) within range
//######################################################################

road.prototype.getTargetNeighbourhood = function (umin, umax, targetLane) {
	var targetVehicles = [];
	var iTarget = 0;
	var firstTime = true;
	for (var i = 0; i < this.veh.length; i++) {
		if ((this.veh[i].lane == targetLane) && (this.veh[i].u >= umin) && (this.veh[i].u <= umax)) {
			if (firstTime == true) { this.iOffset = i; firstTime = false; }
			targetVehicles[iTarget] = this.veh[i];
			iTarget++;
		}
	}
	return targetVehicles;
}


//####################################################
// distribute model parameters updated from  GUI to all vehicles
//####################################################

road.prototype.updateModelsOfAllVehicles = function (longModelCar, longModelTruck, LCModelCar, LCModelTruck) {

	this.nveh = this.veh.length; // just in case; this is typically first cmd for update
	this.updateSwarmSpeed();
	for (var i = 0; i < this.nveh; i++) {
		if (this.veh[i].type != "obstacle") {// then do nothing
			if(this.veh[i].type == "ambulance") continue;
			this.veh[i].longModel = (this.veh[i].type == "car")
				? longModelCar : longModelTruck;
			this.veh[i].LCModel = (this.veh[i].type == "car")
				? LCModelCar : LCModelTruck;
		}
	}

}

//######################################################################
// update times since last change for all vehicles (min time between changes)
//######################################################################

road.prototype.updateLastLCtimes = function (dt) {
	for (var i = 0; i < this.nveh; i++) {
		this.veh[i].dt_lastLC += dt;
		this.veh[i].dt_lastPassiveLC += dt;
	}
}
//######################################################################
// get direction of road at arclength u
//######################################################################
/**
@param traj_x(u), traj_y(u)=phys. road geometry as parametrized 
	   function of the arc length
@param u=actual arclength for which to get direction
@return direction (heading) of the road (0=East, pi/2=North etc)
*/

road.prototype.get_phi = function (traj_x, traj_y, u) {

	var smallVal = 0.0000001;

	var du = 0.1;
	var dx = traj_x(u + du) - traj_x(u - du);
	var dy = traj_y(u + du) - traj_y(u - du);
	var phi = (Math.abs(dx) < smallVal) ? 0.5 * Math.PI : Math.atan(dy / dx);
	if ((dx < 0) || ((Math.abs(dx) < smallVal) && (dy < 0))) { phi += Math.PI; }
	return phi;
}


road.prototype.draw = function (roadImg, scale, traj_x, traj_y, laneWidth) {

	var smallVal = 0.0000001;
	var boundaryStripWidth = 0.3 * laneWidth;

	var factor = 1 + this.nLanes * laneWidth * this.draw_curvMax; // "stitch factor"
	var lSegm = this.roadLen / this.draw_nSegm;

	// only at beginning or after rescaling

	if (Math.abs(scale - this.draw_scaleOld) > smallVal) {
		this.draw_scaleOld = scale;
		for (var iSegm = 0; iSegm < this.draw_nSegm; iSegm++) {
			var u = this.roadLen * (iSegm + 0.5) / this.draw_nSegm;
			this.draw_x[iSegm] = traj_x(u);
			this.draw_y[iSegm] = traj_y(u);
			this.draw_phi[iSegm] = this.get_phi(traj_x, traj_y, u);
			this.draw_cosphi[iSegm] = Math.cos(this.draw_phi[iSegm]);
			this.draw_sinphi[iSegm] = Math.sin(this.draw_phi[iSegm]);

		}
	}

	// actual drawing routine

	for (var iSegm = 0; iSegm < this.draw_nSegm; iSegm++) {
		var cosphi = this.draw_cosphi[iSegm];
		var sinphi = this.draw_sinphi[iSegm];
		var lSegmPix = scale * factor * lSegm;
		//var lSegmPix=scale*1*lSegm;
		var wSegmPix = scale * (this.nLanes * laneWidth + boundaryStripWidth);

		var vCenterPhys = 0.0 * this.nLanes * laneWidth; //check if not =0 if traj at center!!
		var xCenterPix = scale * (this.draw_x[iSegm] + vCenterPhys * sinphi);
		var yCenterPix = -scale * (this.draw_y[iSegm] - vCenterPhys * cosphi);
		ctx.setTransform(cosphi, -sinphi, +sinphi, cosphi, xCenterPix, yCenterPix);
		ctx.drawImage(roadImg, -0.5 * lSegmPix, -0.5 * wSegmPix, lSegmPix, wSegmPix);
		
	}
}// draw road


road.prototype.drawVehicles = function (
    carImg, truckImg, obstacleImg, scale, traj_x, traj_y, laneWidth, 
    speedmin, speedmax, umin, umax
) {
    var noRestriction = (typeof umin === 'undefined');
    let colors = ["red", "blue", "green", "orange", "purple", "cyan", "pink"];

    for (var i = 0; i < this.veh.length; i++) {
        if (noRestriction || ((this.veh[i].u >= umin) && (this.veh[i].u <= umax))) {
            var veh = this.veh[i];
            var type = veh.type;
            var vehLenPix = scale * veh.length;
            var vehWidthPix = scale * veh.width;
            var uCenterPhys = veh.u - 0.5 * veh.length;
            var vCenterPhys = laneWidth * (veh.v - 0.5 * (this.nLanes - 1));

            var phiRoad = this.get_phi(traj_x, traj_y, uCenterPhys);
            var phiVehRel = (Math.abs(veh.dvdu) < 0.00001) ? 0 : -Math.atan(veh.dvdu);
            var phiVeh = phiRoad + phiVehRel;
            var cphiRoad = Math.cos(phiRoad);
            var sphiRoad = Math.sin(phiRoad);
            var cphiVeh = Math.cos(phiVeh);
            var sphiVeh = Math.sin(phiVeh);
            var xCenterPix = scale * (traj_x(uCenterPhys) + vCenterPhys * sphiRoad);
            var yCenterPix = -scale * (traj_y(uCenterPhys) - vCenterPhys * cphiRoad);

            // (1) Draw vehicle image
            vehImg = (type == "car") ? veh.image : (type == "truck") ? truckImg : obstacleImg;
            ctx.setTransform(cphiVeh, -sphiVeh, +sphiVeh, cphiVeh, xCenterPix, yCenterPix);
            ctx.drawImage(vehImg, -0.5 * vehLenPix, -0.5 * vehWidthPix, vehLenPix, vehWidthPix);

            // (2) Highlight vehicles in a swarm
            if (veh.swarmId !== undefined) {
                let swarmColor = colors[veh.swarmId % colors.length];

                // (A) Draw swarm outline
                ctx.lineWidth = 2;
                ctx.strokeStyle = swarmColor;
                ctx.strokeRect(-0.5 * vehLenPix, -0.5 * vehWidthPix, vehLenPix, vehWidthPix);

                // (B) Add glow effect
                ctx.shadowColor = swarmColor;
                ctx.shadowBlur = 10;
                ctx.stroke();

                // Reset glow after applying
                ctx.shadowColor = "transparent";
                ctx.shadowBlur = 0;

                // (C) Display swarm ID
                // ctx.font = "bold 14px Arial";
                // ctx.fillStyle = "white";
                // ctx.textAlign = "center";
                // ctx.fillText(`S${veh.swarmId}`, 0, -vehWidthPix / 2 - 5);
            }


            // (4) Flashing lights for Ambulance
            if (type === "ambulance") {
                var lightColor = (Math.floor(Date.now() / 300) % 2 === 0) ? "red" : "blue";
                ctx.fillStyle = lightColor;
                ctx.fillRect(-0.1 * vehLenPix, -0.2 * vehWidthPix, 7, 7);
            }
        }
    }
};





road.prototype.formSwarms = function () {
    let assigned = new Set();
    this.swarms = [];
    let swarmId = 0;

    for (let i = 0; i < this.nveh; i++) {
        let veh = this.veh[i];

        // 🚑 **Skip ambulance** completely
        if (veh.isAmbulance) {
            continue; // 🔥 Ensures ambulance is ignored
        }
		if(veh.type == "obstacle"){
			continue;
		}

        // 🚗 Check if already assigned
        if (assigned.has(veh)) {
            continue; 
        }

        // Create new swarm
        let swarm = [veh];
        veh.swarmId = swarmId;

        for (let j = 0; j < this.nveh; j++) {
            if (i !== j && !assigned.has(this.veh[j])) {
                let otherVeh = this.veh[j];

                // 🚑 **Ensure ambulance does not join**
                if (otherVeh.isAmbulance) {
                    continue; 
                }

				if(otherVeh.type == "obstacle"){
					continue;
				}

                let dist = Math.abs(otherVeh.u - veh.u);
                if (dist < 100) { // Vehicles within 100m join swarm
                    swarm.push(otherVeh);
                    otherVeh.swarmId = swarmId;
                    assigned.add(otherVeh);
                }
            }
        }
        this.swarms.push(swarm);
        assigned.add(veh);
        swarmId++;
    }
};

road.prototype.updateSwarmSpeed = function () {
	console.log(this.swarms)
}


road.prototype.callAmbulance = function (startLane, startPos, speed) {
    let ambulance = new Vehicle(10, laneWidth-2, startPos, startLane, speed, "ambulance");
	ambulance.route = 0;
	this.ambulances.push(ambulance);
    // 🚑 Set IDM for emergency driving (Higher speed, faster acceleration)
    ambulance.longModel = new IDM(130, 0.8, 1, 3, 3);  
    // v0=40, T=0.8s, s0=1m, a=3m/s², b=3m/s²
    // 🚑 Set MOBIL for aggressive lane-changing (lower safety threshold)
    ambulance.LCModel = new MOBIL(6, 0.1, 0.8);  
    // bSafe=6, bThr=0.1, bBiasRight=0.8 (eager to change lanes)
	ambulance.isAmbulance = true;
    this.veh.push(ambulance);

    console.log("🚑 Ambulance added at lane " + startLane + ", position " + startPos);

    // Make swarm vehicles react
    return ambulance;
};



// road.prototype.reactToAmbulance = function () {

//     for (let i = 0; i < this.nveh; i++) {
//         let veh = this.veh[i];

//         // 🛑 Ignore non-ambulance vehicles
//         if (!veh.isAmbulance) continue;

//         let ambulance = veh;
//         let lane = ambulance.lane;
//         let position = ambulance.u;

//         console.log(`🚑 Ambulance detected at u=${position}, lane=${lane}`);

//         for (let j = 0; j < this.nveh; j++) {
//             let otherVeh = this.veh[j];

//             if (otherVeh === ambulance) continue; // 🚨 Skip self

//             let otherLane = otherVeh.lane;
//             let otherPos = otherVeh.u;
//             let distance = otherPos - position;

//             // 🚗 Vehicles in front of ambulance accelerate
//             if (otherLane === lane && distance > 0 && distance < 50 && veh.type !== "obstacle") {
//                 otherVeh.speed += 2; // Accelerate by +2
//                 console.log(`🚀 Vehicle at u=${otherPos} speeds up!`);
//             }
//         }
//     }
// };


road.prototype.isLaneClear = function (veh, targetLane) {
    for (let i = 0; i < this.nveh; i++) {
        let otherVeh = this.veh[i];

        if (otherVeh === veh) continue; // Skip self

        let laneDistance = Math.abs(otherVeh.lane - targetLane);
        let posDistance = Math.abs(otherVeh.u - veh.u);

        // If another vehicle is too close in the target lane, return false
        if (laneDistance === 0 && posDistance < 10) {
            return false;
        }
    }
    return true;
};
