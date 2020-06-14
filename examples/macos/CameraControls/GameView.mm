//
//  GameView.m
//  CameraControls
//
//  Created by Nick Porcino on 2/20/15.
//  Copyright (c) 2015 Planet IX. All rights reserved.
//

#import "GameView.h"
#import <SpriteKit/SpriteKit.h>
#include <algorithm>

/*
#include "ImathEuler.h"
#include "ImathMatrixAlgo.h"
#include "ImathQuat.h"
*/

#define LAB_CAMERA_DRY
#include "LabCamera.h"

typedef vector_float3 v3f;
typedef vector_float4 v4f;

static float planeIntersect(v3f rayDirection,
                            v3f rayOrigin,
                            v3f planePoint,
                            v3f planeNormal) {
    float denom = vector_dot(planeNormal, rayDirection);
    if (denom > 1.e-6f) {
        v3f p0 = planePoint - rayOrigin;
        return vector_dot(p0, planeNormal) / denom;
    }
    return FLT_MAX; // ray and plane are parallel
}

static matrix_float4x4 lookat(v3f eye, v3f target, v3f up) {
    v3f zaxis = vector_normalize(eye - target);
    v3f xaxis = vector_normalize(vector_cross(up, zaxis));
    v3f yaxis = vector_cross(zaxis, xaxis);
    matrix_float4x4 ret = { (vector_float4){xaxis.x, yaxis.x, zaxis.x, 0},
                            (vector_float4){xaxis.y, yaxis.y, zaxis.y, 0},
                            (vector_float4){xaxis.z, yaxis.z, zaxis.z, 0},
                            (vector_float4){-vector_dot(xaxis, eye), -vector_dot(yaxis, eye), -vector_dot(zaxis, eye), 1 }};
    return ret;
}


@implementation GameView {
    SCNNode* clickPoint;
    
    v3f initialCameraPosition;
    vector_float2 initial2dClickPosition;
    vector_float2 previousMousePosition;
    
    float constrainedCameraDistanceFromPlane;
    SKScene *HUD;
    
    SKSpriteNode* indicator;
    
    lab::camera::CameraRigMode cameraMode;
    lab::camera::Camera camera;
}

-(void) createHUD {
    HUD = [SKScene new];
    HUD.anchorPoint = CGPointMake(0.5, 0.5);
    HUD.scaleMode = SKSceneScaleModeResizeFill;
    
    float startX = -80;
    float startY = self.frame.size.height * 0.5f - 50;
    SKLabelNode* label = [SKLabelNode labelNodeWithFontNamed:@"Avenir"];
    label.text = @"Center";
    label.name = @"look-at";
    label.fontSize = 20.f;
    label.color = [SKColor yellowColor];
    label.position = CGPointMake(startX + 0, startY);
    label.horizontalAlignmentMode = SKLabelHorizontalAlignmentModeLeft;
    [HUD addChild:label];
    label = [SKLabelNode labelNodeWithFontNamed:@"Avenir"];
    label.text = @"Crane";
    label.name = @"crane";
    label.fontSize = 20.f;
    label.color = [SKColor yellowColor];
    label.position = CGPointMake(startX + 82, startY);
    label.horizontalAlignmentMode = SKLabelHorizontalAlignmentModeLeft;
    [HUD addChild:label];

    label = [SKLabelNode labelNodeWithFontNamed:@"Avenir"];
    label.text = @"Orbit";
    label.name = @"turntable-orbit";
    label.fontSize = 20.f;
    label.color = [SKColor yellowColor];
    label.position = CGPointMake(startX + 155, startY);
    label.horizontalAlignmentMode = SKLabelHorizontalAlignmentModeLeft;
    [HUD addChild:label];
    indicator = [SKSpriteNode spriteNodeWithColor:[SKColor yellowColor] size:CGSizeMake(8,8)];
    indicator.position = CGPointMake(label.position.x - 5, label.position.y + 5);
    [HUD addChild:indicator];
    
    label = [SKLabelNode labelNodeWithFontNamed:@"Avenir"];
    label.text = @"Dolly";
    label.name = @"dolly";
    label.fontSize = 20.f;
    label.color = [SKColor yellowColor];
    label.position = CGPointMake(startX + 220, startY);
    label.horizontalAlignmentMode = SKLabelHorizontalAlignmentModeLeft;
    [HUD addChild:label];
    
    self.overlaySKScene = HUD;
    cameraMode = lab::camera::CameraRigMode::TurnTableOrbit;
    self->previousMousePosition = {0,0};

    camera.position = { (float) self.pointOfView.position.x, (float) self.pointOfView.position.y, (float) self.pointOfView.position.z };
    camera.focusPoint = { 0, 0, 0 };
}

-(v3f) viewRay:(NSPoint*)pointInWindow {
    v3f cameraLoc = SCNVector3ToFloat3(
                                    [self unprojectPoint:SCNVector3Make(pointInWindow->x, pointInWindow->y, 0.f)]);
    v3f forwardLoc = SCNVector3ToFloat3(
                                    [self unprojectPoint:SCNVector3Make(pointInWindow->x, pointInWindow->y, 1.f)]);
    return vector_normalize(forwardLoc - cameraLoc);
}

-(void) mouseDown:(NSEvent *)theEvent {
    
    CGPoint p = [HUD convertPointFromView:[theEvent locationInWindow]];

    NSPoint click2d = [self convertPoint:[theEvent locationInWindow] fromView:nil];
    self->initial2dClickPosition = {(float)click2d.x, (float)click2d.y};
    self->previousMousePosition = self->initial2dClickPosition;
    
    
    
    
    
    
/////////////////    2. eliminate planeIntersect with camera.planeIntersect.

    
    
    
    
    
    v3f cameraPos = { camera.position.x, camera.position.y, camera.position.z };
    
    SKNode* clickedSprite = [HUD nodeAtPoint:p];
    if (clickedSprite != nil) {
        if ([clickedSprite.name isEqualTo:@"dolly"]) {
            self->cameraMode = lab::camera::CameraRigMode::Dolly;
            indicator.position = CGPointMake(clickedSprite.position.x - 5, clickedSprite.position.y + 5);
            self.needsDisplay = YES;
        }
        else if ([clickedSprite.name isEqualTo:@"turntable-orbit"]) {
            self->cameraMode = lab::camera::CameraRigMode::TurnTableOrbit;
            indicator.position = CGPointMake(clickedSprite.position.x - 5, clickedSprite.position.y + 5);
            self.needsDisplay = YES;
        }
        else if ([clickedSprite.name isEqualTo:@"crane"]) {
            self->cameraMode = lab::camera::CameraRigMode::Crane;
            indicator.position = CGPointMake(clickedSprite.position.x - 5, clickedSprite.position.y + 5);
            self.needsDisplay = YES;
        }
        else if ([clickedSprite.name isEqualTo:@"look-at"]) {
            simd_float3 pnt = { self->camera.focusPoint.x, self->camera.focusPoint.y, self->camera.focusPoint.z };
            self.pointOfView.transform = SCNMatrix4FromMat4(matrix_invert(lookat(cameraPos, pnt, (vector_float3){0,1,0})));
        }
        NSLog(@"%@", clickedSprite.name? clickedSprite.name : @"Unnamed sprite clicked");
    }
    
    if (!clickPoint) {
        SCNSphere *sphere = [SCNSphere sphereWithRadius:0.1f];
        clickPoint = [SCNNode new];
        clickPoint.geometry = sphere;
        [self.scene.rootNode addChildNode:clickPoint];
        clickPoint.hidden = YES;
    }

    if (clickPoint.hidden) {
        // test for clicked object
        //
        NSPoint p = [self convertPoint:[theEvent locationInWindow] fromView:nil];
        NSArray *hitResults = [self hitTest:NSPointToCGPoint(p) options:nil];
        
        if ([hitResults count] > 0) {
            
            SCNHitTestResult *result = [hitResults objectAtIndex:0];
            
            SCNVector3 worldPos = result.worldCoordinates;
            simd_float3 pnt = SCNVector3ToFloat3(worldPos);
            self->camera.focusPoint = {pnt.x, pnt.y, pnt.z};
            clickPoint.position = worldPos;
            self.pointOfView.transform = SCNMatrix4FromMat4(matrix_invert(lookat(cameraPos, pnt, (vector_float3){0,1,0})));
        }
        clickPoint.hidden = NO;
    }
    else {
       // clickPoint.hidden = YES;
    }
    
    initialCameraPosition = { camera.position.x, camera.position.y, camera.position.z };

    NSPoint zero = {0,0};
    v3f forwardRay = [self viewRay:&zero];
    v3f planePoint = SCNVector3ToFloat3(clickPoint.position);
    v3f viewOrigin = initialCameraPosition;
    constrainedCameraDistanceFromPlane = planeIntersect(forwardRay, viewOrigin, planePoint, forwardRay);
        
    [super mouseDown:theEvent];
}

-(void) mouseDragged:(NSEvent *)theEvent {
    NSPoint p = [self convertPoint:[theEvent locationInWindow] fromView:nil];
    
    vector_float2 currentClickPosition = {(float)p.x, (float)p.y};
    
    v3f upVector = {
        (float)self.pointOfView.transform.m21,
        (float)self.pointOfView.transform.m22,
        (float)self.pointOfView.transform.m23};
    upVector = vector_normalize(upVector);
    v3f fwdVector = {
        (float)self.pointOfView.transform.m31,
        (float)self.pointOfView.transform.m32,
        (float)self.pointOfView.transform.m33};
    fwdVector = vector_normalize(fwdVector);
    v3f rightVector = {
        (float)self.pointOfView.transform.m11,
        (float)self.pointOfView.transform.m12,
        (float)self.pointOfView.transform.m13};
    rightVector = vector_normalize(rightVector);

    //v3f cameraPos = { camera.position.x, camera.position.y, camera.position.z };
    vector_float2 delta = currentClickPosition - self->previousMousePosition;
    //float distanceY = 2.f * delta.y / self.frame.size.height;
    //float distanceX = 2.f * delta.x / self.frame.size.width;

    //v3f focusPnt = {self->camera.focusPoint.x, self->camera.focusPoint.y, self->camera.focusPoint.z};
    //float distanceToFocus = vector_length(cameraPos - focusPnt);
    //float scale = std::max(0.01f, logf(distanceToFocus) * 5.f);
    
    if (cameraMode == lab::camera::CameraRigMode::Dolly) {
#if 1
        lab::camera::cameraRig_interact(camera, cameraMode, {delta.x, delta.y});
        lab::camera::m44f t = camera.mount.viewTransform();
        simd_float4x4 m_inv = matrix_invert(*((simd_float4x4*)&t));
        self.pointOfView.transform = SCNMatrix4FromMat4(m_inv);
#else
        v3f dP = distanceY * fwdVector * scale - distanceX * rightVector * scale;
        cameraPos += dP;
        focusPnt += dP;
        self->camera.focusPoint = { focusPnt.x, focusPnt.y, focusPnt.z };
        clickPoint.position = SCNVector3FromFloat3(focusPnt);
        self.pointOfView.position = SCNVector3FromFloat3(cameraPos);
        camera.position = { cameraPos.x, cameraPos.y, cameraPos.z };
#endif
    }
    if (cameraMode == lab::camera::CameraRigMode::Crane) {
#if 1
        lab::camera::cameraRig_interact(camera, cameraMode, {delta.x, delta.y});
        lab::camera::m44f t = camera.mount.viewTransform();
        simd_float4x4 m_inv = matrix_invert(*((simd_float4x4*)&t));
        self.pointOfView.transform = SCNMatrix4FromMat4(m_inv);
#else
        v3f dP = -distanceY * upVector * scale - distanceX * rightVector * scale;
        cameraPos += dP;
        focusPnt += dP;
        self->camera.focusPoint = { focusPnt.x, focusPnt.y, focusPnt.z };
        clickPoint.position = SCNVector3FromFloat3(focusPnt);
        self.pointOfView.position = SCNVector3FromFloat3(cameraPos);
        camera.position = { cameraPos.x, cameraPos.y, cameraPos.z };
#endif
    }
    else if (cameraMode == lab::camera::CameraRigMode::TurnTableOrbit) {
#if 1
        lab::camera::cameraRig_interact(camera, cameraMode, {delta.x, delta.y});
        lab::camera::m44f t = camera.mount.viewTransform();
        simd_float4x4 m_inv = matrix_invert(*((simd_float4x4*)&t));
        self.pointOfView.transform = SCNMatrix4FromMat4(m_inv);
#else
        v3f turntable_up = {0,1,0};
        v3f mUv = vector_normalize(vector_cross(turntable_up, fwdVector));
        Imath::V3f mU(mUv.x, mUv.y, mUv.z);
        
        distanceX *= -1;
        
        v3f delta = cameraPos - focusPnt;

        Imath::Quatf yaw;
        yaw.setAxisAngle(Imath::V3f(0,1,0), distanceX);
        Imath::Quatf pitch;
        pitch.setAxisAngle(mU, distanceY);

        Imath::V3f rotatedVec = yaw.rotateVector(pitch.rotateVector(Imath::V3f(delta.x, delta.y, delta.z)));
        cameraPos = focusPnt + (vector_float3){rotatedVec.x, rotatedVec.y, rotatedVec.z};

        self.pointOfView.transform = SCNMatrix4FromMat4(matrix_invert(lookat(cameraPos, focusPnt, turntable_up)));
        camera.position = { cameraPos.x, cameraPos.y, cameraPos.z };
#endif
    }
    
    self->previousMousePosition = currentClickPosition;
}

@end
