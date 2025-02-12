function randomCarImage() {
    const carImages = ["car1.png", "car2.png", "car3.png", "car4.png"];
    return carImages[Math.floor(Math.random() * carImages.length)];
}

class Vehicle {
    static imageCache = {}; // Cache for storing images

    static getCarImage() {
        const carImgSrc = 'figs/' + randomCarImage();
        if (!Vehicle.imageCache[carImgSrc]) {
            let img = new Image();
            img.src = carImgSrc;
            Vehicle.imageCache[carImgSrc] = img;
        }
        return Vehicle.imageCache[carImgSrc];
    }

    constructor(length, width, u, lane, speed, type) {
        this.length = length;
        this.width = width;
        this.u = u;
        this.lane = lane;
        this.speed = speed;
        this.type = type;
        this.image = Vehicle.getCarImage();

        this.route = []; // Sequence of road IDs
        this.mandatoryLCahead = false;
        this.toRight = false;

        this.isAmbulance = false;

        this.v = lane; // v = lane coordinate, not speed!!
        this.dvdu = lane;
        this.laneOld = lane;
        this.dt_lastLC = 10;
        this.dt_lastPassiveLC = 10;
        this.acc = -100;

        this.iLead = -100;
        this.iLag = -100;
        this.iLeadOld = -100;
        this.iLagOld = -100;
        this.iLeadRight = -100;
        this.iLeadLeft = -100;
        this.iLagRight = -100;
        this.iLagLeft = -100;
        this.iLeadRightOld = -100;
        this.iLeadLeftOld = -100;
        this.iLagRightOld = -100;
        this.iLagLeftOld = -100;
        
        // Driving models
        this.longModel = new IDM(20, 1.3, 2, 1, 2);
        this.LCModel = new MOBIL(4, 6, 0.2, 0.3); // Fixed missing parameter
    }

    setRoute(route) {
        if (!Array.isArray(route)) {
            throw new Error("Route must be an array of road IDs.");
        }
        this.route = route;
    }
}
