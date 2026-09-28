{
 "name":"rebuilt2026",
 "version":2,
 "type":"Swerve",
 "variables":{
  "expressions":{
   "DepotCenterY":{
    "dimension":"Length",
    "var":{
     "exp":"FieldCenterY + 75.93 in",
     "val":5.963285
    }
   },
   "FieldCenterX":{
    "dimension":"Length",
    "var":{
     "exp":"FieldLength / 2",
     "val":8.270494
    }
   },
   "FieldCenterY":{
    "dimension":"Length",
    "var":{
     "exp":"FieldWidth / 2",
     "val":4.034663
    }
   },
   "FieldLength":{
    "dimension":"Length",
    "var":{
     "exp":"651.22 in",
     "val":16.540988
    }
   },
   "FieldWidth":{
    "dimension":"Length",
    "var":{
     "exp":"317.69 in",
     "val":8.069326
    }
   },
   "HubCenterX":{
    "dimension":"Length",
    "var":{
     "exp":"182.11 in",
     "val":4.625594
    }
   },
   "IntakeClearance":{
    "dimension":"Length",
    "var":{
     "exp":"RobotLength / 2 + 13 in",
     "val":0.70485
    }
   },
   "MaxAcceleration":{
    "dimension":"LinAcc",
    "var":{
     "exp":"5 m / s ^ 2",
     "val":5.0
    }
   },
   "MaxVelocity":{
    "dimension":"LinVel",
    "var":{
     "exp":"3.75 m / s",
     "val":3.75
    }
   },
   "RobotLength":{
    "dimension":"Length",
    "var":{
     "exp":"29.5 in",
     "val":0.7493
    }
   },
   "RobotWidth":{
    "dimension":"Length",
    "var":{
     "exp":"37.5 in",
     "val":0.9525
    }
   },
   "TrenchShotWallOffset":{
    "dimension":"Length",
    "var":{
     "exp":"5.5 in",
     "val":0.1397
    }
   }
  },
  "poses":{
   "LeftTrenchStart":{
    "x":{
     "exp":"4.05 m",
     "val":4.05
    },
    "y":{
     "exp":"FieldWidth - (RobotWidth / 2) - TrenchShotWallOffset",
     "val":7.453376
    },
    "heading":{
     "exp":"0 rad",
     "val":0.0
    }
   },
   "RightTrenchStart":{
    "x":{
     "exp":"LeftTrenchStart.x",
     "val":4.05
    },
    "y":{
     "exp":"(RobotWidth / 2) + TrenchShotWallOffset",
     "val":0.61595
    },
    "heading":{
     "exp":"0 rad",
     "val":0.0
    }
   }
  }
 },
 "config":{
  "frontLeft":{
   "x":{
    "exp":"18 in / 2",
    "val":0.2286
   },
   "y":{
    "exp":"26.5 in / 2",
    "val":0.33654999999999996
   }
  },
  "backLeft":{
   "x":{
    "exp":"-18 in / 2",
    "val":-0.2286
   },
   "y":{
    "exp":"26.5 in / 2",
    "val":0.33654999999999996
   }
  },
  "mass":{
   "exp":"115 lbs",
   "val":52.16312255
  },
  "inertia":{
   "exp":"6 kg m ^ 2",
   "val":6.0
  },
  "gearing":{
   "exp":"7.67",
   "val":7.67
  },
  "radius":{
   "exp":"2 in",
   "val":0.0508
  },
  "vmax":{
   "exp":"6000 RPM",
   "val":628.3185307179587
  },
  "tmax":{
   "exp":"1.2 N * m",
   "val":1.2
  },
  "cof":{
   "exp":"1.5",
   "val":1.5
  },
  "bumper":{
   "front":{
    "exp":"IntakeClearance",
    "val":0.70485
   },
   "side":{
    "exp":"RobotWidth / 2",
    "val":0.47625
   },
   "back":{
    "exp":"RobotLength / 2",
    "val":0.37465
   }
  },
  "differentialTrackWidth":{
   "exp":"22 in",
   "val":0.5588
  }
 },
 "generationFeatures":[],
 "codegen":{
  "root":"..\\..\\java\\frc\\robot\\generated\\choreo",
  "genVars":true,
  "genTrajData":true,
  "useChoreoLib":true
 }
}
