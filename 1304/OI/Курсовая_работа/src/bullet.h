//
//  bullet.h
//  RobotSim
//
//  Created by Imhoisili Otokhagua on 13/12/2016.
//  Copyright © 2016 Imhoisili Otokhagua. All rights reserved.
//

#ifndef bullet_h
#define bullet_h

#include "entity.h"

class Bullet : public Entity {
    // positoin
    // velocity
public:
    float range;
    
    void update(float dt) override {
        
    }
};

#endif /* bullet_h */
