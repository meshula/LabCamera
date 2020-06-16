//
//  GameViewController.m
//  CameraControls
//
//  Created by Nick Porcino on 2/20/15.
//  Copyright (c) 2015 Planet IX. All rights reserved.
//

#import "GameViewController.h"

@implementation GameViewController

-(void)awakeFromNib
{
    // create a new scene
    SCNScene *scene = [SCNScene sceneNamed:@"art.scnassets/ship.dae"];

    // create and add a camera to the scene
    SCNNode *cameraNode = [SCNNode node];
    cameraNode.camera = [SCNCamera camera];
    [scene.rootNode addChildNode:cameraNode];
    
    // place the camera
    cameraNode.position = SCNVector3Make(0, 0, 20);
    
    SCNBox *box = [SCNBox boxWithWidth:100 height:1 length:100 chamferRadius:0];
    SCNNode *boxNode = [SCNNode new];
    boxNode.position = SCNVector3Make(0, -10, 0);
    boxNode.geometry = box;
    [scene.rootNode addChildNode:boxNode];
    
    // create and add a light to the scene
    SCNNode *lightNode = [SCNNode node];
    lightNode.light = [SCNLight light];
    lightNode.light.type = SCNLightTypeOmni;
    lightNode.position = SCNVector3Make(0, 10, 10);
    [scene.rootNode addChildNode:lightNode];
    
    // create and add an ambient light to the scene
    SCNNode *ambientLightNode = [SCNNode node];
    ambientLightNode.light = [SCNLight light];
    ambientLightNode.light.type = SCNLightTypeAmbient;
    ambientLightNode.light.color = [NSColor darkGrayColor];
    [scene.rootNode addChildNode:ambientLightNode];
    
    // set the scene to the view
    self.gameView.scene = scene;
    
    // allows SceneKit to mess with the camera
    self.gameView.allowsCameraControl = NO;
    
    // show statistics such as fps and timing information
    self.gameView.showsStatistics = YES;
    
    // configure the view
    self.gameView.backgroundColor = [NSColor blackColor];
    
    [self.gameView createHUD];
}

@end
