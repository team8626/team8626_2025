// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// Original Work by Team 6328 Mechanical Advantage

import { NT4_Client } from "./NT4.js";
// ***** NETWORKTABLES *****

const toRobotPrefix = "/UIDashboard/ToRobot/";
const toDashboardPrefix = "/UIDashboard/ToDashboard/";
const allianceColorTopicName = "allianceColor";
const allowedCoralLevelsTopicName = "allowedCoralLevels";
const selectedCoralLevelTopicName = "selectedCoralLevel";
const selectedCoralBranchTopicName = "selectedCoralBranch";
const selectedAlgaeFaceTopicName = "selectedAlgaeFace";
const selectedPickupSideTopicName = "selectedPickupSide";
const selectedDtpTopicName = "selectedDtp";
const matchTimeTopicName = "matchTime";
const isAutoTopicName = "isAuto";
const algaeStateTopicName = "algaeState";
const coralStateTopicName = "coralState";
const algaeShootTimeTopicName = "algaeShootTime";
const coralShootTimeTopicName = "coralShootTime";
const autoPathTopicName = "/SmartDashboard/Auto Path";

// ***** STATE CACHE *****
let allianceColor = 0; // int (0 = Unknown, 1 = Red, 2 Blue)
let allowedCoralLevels = 0; // int[]
let selectedCoralLevel = 0; // int (1 = L1, 2 = L2, 3 = L3, 4 =L4)
let selectedCoralBranch = 0; // int
let selectedAlgaeFace = 0; // int
let selectedPickupSide = 0; // int (0 = Unselected, 1 = Left, 2 = Right)
let selectedDtp = false; // boolean
let matchTime = 0; // double (seconds)
let isAuto = true; // double (seconds)
let algaeStatus = "---"; // string
let coralStatus = "---"; // string
let algaeShootTime = 0; // double (ms)
let coralShootTime = 0; // double (ms)
let isConnectionActive = false;

const ntClient = new NT4_Client(
  window.location.hostname,
  "ReefControls",
  (topic) => {
    // Topic announce
  },
  (topic) => {
    // Topic unannounce
  },
  (topic, timestamp, value) => {
    // New data
    if (topic.name === toDashboardPrefix + allianceColorTopicName) {
      allianceColor = value;
      udateAlliance(allianceColor);
      console.log("Rx Alliance color: " + allianceColor);
    } else if (topic.name === toDashboardPrefix + isAutoTopicName) {
      isAuto = value;
      updateMatchTime(matchTime, isAuto);
      console.log("Rx Is Auto: " + isAuto);
    } else if (topic.name === toDashboardPrefix + matchTimeTopicName) {
      matchTime = value;
      updateMatchTime(matchTime, isAuto);
    } else if (topic.name === toDashboardPrefix + allowedCoralLevelsTopicName) {
      allowedCoralLevels = value;
      console.log("Rx Allowed coral levels: " + allowedCoralLevels);
    } else if (topic.name === toDashboardPrefix + selectedCoralLevelTopicName) {
      selectedCoralLevel = value;
      updateCoralLevelValue(selectedCoralLevel, true);
      console.log("Rx Selected coral level: " + selectedCoralLevel);
    } else if (topic.name === toDashboardPrefix + selectedCoralBranchTopicName) {
      selectedCoralBranch = value;
      console.log("Rx Selected coral branch: " + selectedCoralBranch);
      updateCoralBranchValue(selectedCoralBranch, true)
    } else if (topic.name === toDashboardPrefix + selectedAlgaeFaceTopicName) {
      selectedAlgaeFace = value;
      console.log("Rx Selected algae face: " + selectedAlgaeFace);
      updateAlgaeFaceValue(selectedAlgaeFace, true);
    } else if (topic.name === toDashboardPrefix + selectedPickupSideTopicName) {
      selectedPickupSide = value;
      updateIntakeSideValue(selectedPickupSide);
      console.log("Rx Selected pickup side: " + selectedPickupSide);
    } else if (topic.name === toDashboardPrefix + selectedDtpTopicName) {
      selectedDtp = value;
      updateDtpValue(selectedDtp);
      console.log("Rx Input DTP: " + selectedDtp);
    } else if (topic.name === toDashboardPrefix + algaeStateTopicName) {
      algaeStatus = value;
      updateAlgaeState(algaeStatus);
      console.log("Rx Algae state: " + algaeStatus);
    } else if (topic.name === toDashboardPrefix + coralStateTopicName) { 
      coralStatus = value;
      updateCoralState(coralStatus);
      console.log("Rx Coral state: " + coralStatus);
    } else if (topic.name === toDashboardPrefix + algaeShootTimeTopicName) {
      algaeShootTime = value;
      updateAlgaeShootTime(algaeShootTime);
      console.log("Rx Algae shoot time: " + algaeShootTime);
    } else if (topic.name === toDashboardPrefix + coralShootTimeTopicName) {
      coralShootTime = value;
      updateCoralShootTime(coralShootTime);
      console.log("Rx Coral shoot time: " + coralShootTime);
    } else if (topic.name === autoPathTopicName) {
      console.log("Rx Auto Path: " + value);
      alert("Auto Path: " + value);
    } 
    else {
      return;
    }
    updateUI();
  },
  () => {
    // Connected
    if(!isConnectionActive) {
      $("body").css("background-color", "#111529");
      isConnectionActive = true;
    }
  },
  () => {
    // Disconnected
    if(isConnectionActive) {
      $("body").css("background-color", "red"); 
      isConnectionActive = false;
    }
  }
);

// Start NT connection
window.addEventListener("load", () => {
  ntClient.subscribe(
    [
      toDashboardPrefix + allianceColorTopicName,
      toDashboardPrefix + allowedCoralLevelsTopicName,
      toDashboardPrefix + selectedCoralLevelTopicName,
      toDashboardPrefix + selectedCoralBranchTopicName,
      toDashboardPrefix + selectedAlgaeFaceTopicName,
      toDashboardPrefix + selectedPickupSideTopicName,
      toDashboardPrefix + selectedDtpTopicName,
      toDashboardPrefix + matchTimeTopicName,
      toDashboardPrefix + isAutoTopicName,
      toDashboardPrefix + algaeStateTopicName,
      toDashboardPrefix + coralStateTopicName,
      toDashboardPrefix + algaeShootTimeTopicName,
      toDashboardPrefix + coralShootTimeTopicName,
      autoPathTopicName
    ],
    false,
    false,
    0.02
  );

  ntClient.publishTopic(toRobotPrefix + selectedCoralLevelTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + selectedCoralBranchTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + selectedAlgaeFaceTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + selectedPickupSideTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + selectedDtpTopicName, "boolean");
  ntClient.connect();
});

/** Update the full UI based on the state cache. */
function updateUI() {
  // Update alliance color
  // TODO: Update alliance color
  
  // Update allowed coral levels
  //


  // Update selected coral level
  //

  // Update selected coral branch
  //

  // Update selected algae face
  //

  // Update selected pickup side
  //

  // Update selected DTP
  // updateDtpValue(selectedDtp);
}

/**
 * Function to update the DTP value and radio buttons
 * @param {*} newValue 
 */
function updateDtpValue(newValue) {
  $("input[name='drivetopose'] + label").css("background", "");

  // Update the radio buttons and labels based on the new value
  if (selectedDtp === newValue) {
      $("label[for='drivetopose_" + String(newValue) + "']").css("background", "red");
  } else {
      $("input[name='drivetopose'][id='drivetopose_" + newValue + "']").prop('checked', true);
      $("label[for='drivetopose_" + String(newValue) + "']").css("background", "red");
      // currentDtpValue = newValue;
      selectedDtp = newValue;
  }
  // Send the updated value over NetworkTables
  sendBooleanToRobot(selectedDtpTopicName, selectedDtp);
}

/**
 * Function to update the intake side value and radio buttons
 * @param {*} newValue
 */
function updateIntakeSideValue(newValue) {
    // $("input[name='intakeside'] + label").css("background", "");

    if(selectedPickupSide === newValue) {
      $("input[name='intakeside']").prop("checked", false); // Uncheck all radios
      $(this).prop('checked', false);
      selectedPickupSide = 0;
    } else {
      $(this).prop('checked', true);
      // $("label[for='intakeside_" + newValue + "']").css("background", "red");
      selectedPickupSide = newValue;
    }
    // Send the updated value over NetworkTables
    sendIntToRobot(selectedPickupSideTopicName, selectedPickupSide);
};

/**
 * Function to update the coral branch value and radio buttons
 * @param {*} newValue 
 */
function updateCoralBranchValue(newValue, fromRobot = false) {
  if(selectedCoralBranch == newValue && !fromRobot) {
    $("input[name='coralGroup']").prop("checked", false); // Uncheck all radios
    selectedCoralBranch = 0;
  } else {
    $(this).prop('checked', true);
    selectedCoralBranch = newValue;
  }

  if(fromRobot){
    updateCoralTarget(newValue, selectedCoralLevel);
  } else {
    sendIntToRobot(selectedCoralBranchTopicName, selectedCoralBranch);
  }
};

/**
 * Function to update the algae face value and radio buttons
 * @param {*} newValue 
 */
function updateAlgaeFaceValue(newValue, fromRobot = false) {
  if(selectedAlgaeFace == newValue && !fromRobot) {
    $("input[name='algaeGroup']").prop("checked", false); // Uncheck all radios
    selectedAlgaeFace = 0;
  } else {
    $(this).prop('checked', true);
    selectedAlgaeFace = newValue;
  }
  
  if(fromRobot){
    updateAlgaeTarget(newValue);
  } else{
    // Send the updated value over NetworkTables
    sendIntToRobot(selectedAlgaeFaceTopicName, selectedAlgaeFace);
  }
};

/**
 * Function to update the coral level value and radio buttons
 * @param {*} newValue 
 */
function updateCoralLevelValue(newValue, fromRobot = false) {
  if(selectedCoralLevel == newValue) {
    // Do Nothing
  } else {
    $(this).prop('checked', true);
    selectedCoralLevel = newValue;
  }
  if(fromRobot){
    updateCoralTarget(selectedCoralBranch, newValue);
  } else {
    sendIntToRobot(selectedCoralLevelTopicName, selectedCoralLevel);
  }


  // Send the updated value over NetworkTables
  console.log("Sending coral level: " + selectedCoralLevel);
};

/**
 * Function to update Match Time
 * @param {*} newValue
 * @param {*} isAuto
 */
function updateMatchTime(newValue, isAuto) {
  let textelement = $("#matchTimeDisplay");
  let matchtime = $(".match-time");

  if (textelement) {
    // Convert the match time from seconds to minutes and seconds
    const minutes = Math.floor(newValue / 60);
    const seconds = Math.round(newValue % 60);

    // Format the time as mm:ss
    const formattedTime = `${minutes}:${seconds.toString().padStart(2, '0')}`;

    // Update the display
    textelement.text(formattedTime);
  }

  matchtime.removeClass("match-time-autonomous match-time-teleop1 match-time-teleop2 match-time-teleop3");
  if(isAuto){
    matchtime.addClass("match-time-autonomous");
  } else if (newValue > 30 || newValue == 0) {
    matchtime.addClass("match-time-teleop1");
  }
  else if (newValue > 15) {
    matchtime.addClass("match-time-teleop2");
  }
  else {
    matchtime.addClass("match-time-teleop3");
  }
}

function updateAlgaeState(newValue) {
  const textelement = $("#algae-state");

  if (textelement) {
    textelement.text(newValue);
  }
}

function updateCoralState(newValue) {
  const textelement = $("#coral-state");
  if (textelement) {
    textelement.text(newValue);
  }
}

function updateAlgaeTarget(newValue) {
  const textelement = $("#algae-target");
  if (textelement) {
    let target = "Ground"
    if(newValue > 0){
      target  = String.fromCharCode(64+(newValue*2)-1)+ String.fromCharCode(64+(newValue*2));
    
    }
    textelement.text(target);
  }
}

function updateCoralTarget(newBranchValue, newLevelValue) {
  const textelement = $("#coral-target");
  if (textelement) {
    var branch = "--";
    if(newBranchValue > 0){
      branch = String.fromCharCode(64+newBranchValue);
    }
    textelement.text(branch + " - L" + newLevelValue);
  }
}

function updateAlgaeShootTime(newValue) {
  const textelement = $("#algae-shoot-time");

  if (textelement) {
    textelement.text(newValue);
  }
}

function updateCoralShootTime(newValue) {
  const textelement = $("#coral-shoot-time");

  if (textelement) {
    textelement.text(newValue);
  }
}

function udateAlliance(newValue) {
  let hexagon = $(".hexagon-outline"); // Select the hexagon

  // Remove any previous alliance class
  hexagon.removeClass("red-alliance blue-alliance");

  // Add the appropriate class based on `newValue`
  if (newValue === 1) { 
      hexagon.addClass("red-alliance");  // Red Alliance
  } else if (newValue === 2) { 
      hexagon.addClass("blue-alliance"); // Blue Alliance
  } else { 
      // No alliance (keep default styling)
  }
}

/**
 * Send an integer value to the robot
 * @param {*} topic 
 * @param {*} value 
 */
function sendIntToRobot(topic, value) {  
  if (typeof ntClient !== 'undefined' && ntClient.addSample) {
    ntClient.addSample(toRobotPrefix + topic, Number(value));
    console.log("Sending to robot: ", toRobotPrefix + topic, value);
    // ntClient.addSample(toRobotPrefix + topic, value);
  } else {
    console.error("ntClient or ntClient.addSample is not defined");
  }
}

/**
 * Send a boolean value to the robot
 * @param {*} topic 
 * @param {*} value 
 */
function sendBooleanToRobot(topic, value) {  
  // Convert value to a boolean
  let boolValue;
  if (typeof value === "string" && value.toLowerCase() === "false") {
      boolValue = false;
  } else {
      boolValue = Boolean(value);
  }
  if (typeof ntClient !== 'undefined' && ntClient.addSample) {
    ntClient.addSample(toRobotPrefix + topic, boolValue);

    // ntClient.addSample(toRobotPrefix + topic, value);
  } else {
    console.error("ntClient or ntClient.addSample is not defined");
  }
}


// Bind UI buttons
$(document).ready(function () {
  // When any radio button in the drivetopose group is clicked
  $("input[name='drivetopose']").click(function (event) {
    console.log("DTP button clicked");
      updateDtpValue($(this).attr("value"));
  });

  // When any radio button in the intakeside group is clicked
  $("input[name='intakeside']").click(function (event) {
    updateIntakeSideValue($(this).attr("value"));
  });

  // When any radio button in the corallevel group is clicked
  $("input[name='corallevel']").click(function (event) {
    updateCoralLevelValue($(this).attr("value"));
  });

  // When any radio button in the coralGroup group is clicked
  $("input[name='coralGroup']").click(function (event) {
    console.log("Coral: ", $(this).attr("id"));
    updateCoralBranchValue($(this).attr("value"));
  });
  
  // When any radio button in the algaeGroup group is clicked
  $("input[name='algaeGroup']").click(function (event) {
    console.log("Algae: ", $(this).attr("id"));
    updateAlgaeFaceValue($(this).attr("value"));
  });
});