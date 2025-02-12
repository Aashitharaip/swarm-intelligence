class IDM {
	constructor(v0, T, s0, a, b, noiseAcc = 0.3) {
		this.v0 = v0;
		this.T = T;
		this.s0 = s0;
		this.a = a;
		this.b = b;
		this.alpha_v0 = 1;
		this.speedlimit = 1000;
		this.speedmax = 1000;
		this.bmax = 16;
		this.noiseAcc = noiseAcc;
	}

	/**
	 * Calculates acceleration using IDM logic.
	 * @param {number} s - Current spacing to the leading vehicle.
	 * @param {number} v - Current velocity.
	 * @param {number} vl - Velocity of the leading vehicle.
	 * @return {number} - Acceleration value.
	 */
	calcAcc(s, v, vl) {
		let accRnd = this.noiseAcc * (Math.random() - 0.5);
		let v0eff = Math.min(this.v0, this.speedlimit, this.speedmax) * this.alpha_v0;

		if (v0eff <= 0) return 0; // Prevents division issues

		let accFree = v < v0eff 
			? this.a * (1 - Math.pow(v / v0eff, 4)) 
			: this.a * (1 - v / v0eff);

		let safeDistance = Math.sqrt(this.a * this.b) || 1e-6; // Prevent div by zero
		let sstar = this.s0 + v * this.T + 0.5 * v * Math.max(v - vl, 0) / safeDistance;

		let accInt = -this.a * Math.pow(sstar / Math.max(s, this.s0), 2);
		return Math.max(-this.bmax, accFree + accInt + accRnd);
	}
}

class MOBIL {
	constructor(bSafe, bSafeMax, bThr, bBiasRight) {
		this.bSafe = bSafe;
		this.bSafeMax = bSafeMax;
		this.bThr = bThr;
		this.bBiasRight = bBiasRight;
	}

	/**
	 * Determines if a lane change is safe and beneficial.
	 * @param {number} vrel - Relative speed (v/v0).
	 * @param {number} acc - Current acceleration in the current lane.
	 * @param {number} accNew - Expected acceleration in the new lane.
	 * @param {number} accLagTargetNew - Acceleration of the new lane leader.
	 * @param {boolean} toRight - Whether the change is to the right lane.
	 * @return {boolean} - Whether the lane change should occur.
	 */
	realizeLaneChange(vrel, acc, accNew, accLagTargetNew, toRight) {
		let vrelClamped = Math.max(0, Math.min(vrel, 1)); // Clamping for stability
		let bSafeActual = vrelClamped * this.bSafe + (1 - vrelClamped) * this.bSafeMax;
		if (accLagTargetNew < -bSafeActual) return false;
		let laneChangeAdvantage = accNew - acc + this.bBiasRight * (toRight ? 1 : -1) - this.bThr;
		return laneChangeAdvantage > 0;
	}
}

