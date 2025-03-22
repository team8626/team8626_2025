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
const selectedDealgaefyDtpTopicName = "selectedDealgaefyDtp";
const selectedAlgaeShootDtpTopicName = "selectedAlgaeShootDtp";
const matchTimeTopicName = "matchTime";
const isAutoTopicName = "isAuto";
const isTeleopTopicName = "isTeleop";
const algaeStateTopicName = "algaeState";
const coralStateTopicName = "coralState";
const resetCoralBranchTopicName = "resetCoralBranch";
const resetAlgaeFaceTopicName = "resetAlgaeFace";

const algaeShootTimeTopicName = "algaeLastShootTime";
const coralShootTimeTopicName = "coralLastShootTime";
// const autoPathTopicName = "/SmartDashboard/Autos/Pathplanner Trajectory";
const autoPathOptionsTopicName = "/SmartDashboard/Autos/Pathplanner Trajectory/options";
const autoPathActiveTopicName = "/SmartDashboard/Autos/Pathplanner Trajectory/active";
const autoPathSelectedTopicName = "/SmartDashboard/Autos/Pathplanner Trajectory/selected";
// const autoModeTopicName = "/SmartDashboard/Autos/Autonomous Mode";
const autoModeOptionsTopicName = "/SmartDashboard/Autos/Autonomous Mode/options";
const autoModeActiveTopicName = "/SmartDashboard/Autos/Autonomous Mode/active";
const autoModeSelectedTopicName = "/SmartDashboard/Autos/Autonomous Mode/selected";
const commodoreStateTopicName = "/SmartDashboard/Commodore/Current State";

// ***** STATE CACHE *****
let allianceColor = 0; // int (0 = Unknown, 1 = Red, 2 Blue)
let allowedCoralLevels = 0; // int[]
let selectedCoralLevel = 0; // int (1 = L1, 2 = L2, 3 = L3, 4 =L4)
let selectedCoralBranch = 0; // int
let selectedAlgaeFace = 0; // int
let selectedPickupSide = 0; // int (0 = Unselected, 1 = Left, 2 = Right)
let selectedDtp = false; // boolean
let selectedDealgaefyDtp = false; // boolean
let selectedAlgaeShootDtp = false; // boolean
let matchTime = 0; // double (seconds)
let isAuto = false; // boolean
let isTeleop = false; // boolean
let algaeStatus = "---"; // string
let coralStatus = "---"; // string
let algaeShootTime = 0; // double (ms)
let coralShootTime = 0; // double (ms)
let isConnectionSelected = false;
// let selectedAutoMode = "";
// let selectedPath = "";

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
    } else if (topic.name === toDashboardPrefix + isTeleopTopicName) {
      isTeleop = value;
      toggleAutoDropdown(isTeleop);
      console.log("Rx IsTelop: " + isTeleop);
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
    } else if (topic.name === toDashboardPrefix + selectedDealgaefyDtpTopicName) {
      selectedDealgaefyDtp = value;
      updateDealgaefyDtpValue(selectedDealgaefyDtp);
      console.log("Rx Input Dealgaefy DTP: " + selectedDealgaefyDtp);
    } else if (topic.name === toDashboardPrefix + selectedAlgaeShootDtpTopicName) {
      selectedAlgaeShootDtp = value;
      updateAlgaeShootDtpValue(selectedAlgaeShootDtp);
      console.log("Rx Input Shoot Algae DTP: " + selectedAlgaeShootDtp);
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
      updateAlgaeFaceValue(0, true);
      console.log("Rx Algae shoot time: " + algaeShootTime);
    } else if (topic.name === toDashboardPrefix + coralShootTimeTopicName) {
      coralShootTime = value;
      updateCoralShootTime(coralShootTime);
      updateCoralBranchValue(0, true);
      console.log("Rx Coral shoot time: " + coralShootTime);
    } 
    
    // else if (topic.name === autoModeOptionsTopicName) {
    //   console.log("Rx Auto Mode Options: " + value);
    //   populateDropdown("#dropdown-menu-automode", "#selectedAutoMode", value);
    // } else if (topic.name === autoModeActiveTopicName) {
    //   console.log("Rx Auto Mode Selected: " + value);
    //   // selectedAutoMode = value;
    //   updateDropdownSelection("#dropdown-menu-automode", "#selectedAutoMode", value);
    // }
    
    else if (topic.name === commodoreStateTopicName) {
      UpdateRobotState(value);
    } 
    // else if (topic.name === autoPathOptionsTopicName) {
    //   console.log("Rx Auto Path Options: " + value);
    //   populateDropdown("#dropdown-menu-trajectory", "#selectedTrajectory", value);
    // } else if (topic.name === autoPathActiveTopicName) {
    //   console.log("Rx Auto Path Selected: " + value);
    //   // selectedPath = value;
    //   updateDropdownSelection("#dropdown-menu-trajectory", "#selectedTrajectory", value);
    // }
    
    else if (topic.name === toDashboardPrefix + resetCoralBranchTopicName) {
      console.log("Rx Reset Coral Branch (" + value + ")");
      updateCoralBranchValue(0, true, value);
    } else if (topic.name === toDashboardPrefix + resetAlgaeFaceTopicName) {
      console.log("Rx Reset Algae Face (" + value + ")");
      updateAlgaeFaceValue(0, true, value);
    }
    else {
      return;
    }
    updateUI();
  },
  () => {
    // Connected
    if(!isConnectionSelected) {
      $("body").css("background-color", "#111529");
      isConnectionSelected = true;
    }
  },
  () => {
    // Disconnected
    if(isConnectionSelected) {
      $("body").css("background-color", "red"); 
      isConnectionSelected = false;
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
      toDashboardPrefix + selectedDealgaefyDtpTopicName,
      toDashboardPrefix + selectedAlgaeShootDtpTopicName,
      toDashboardPrefix + matchTimeTopicName,
      toDashboardPrefix + isAutoTopicName,
      toDashboardPrefix + isTeleopTopicName,
      toDashboardPrefix + algaeStateTopicName,
      toDashboardPrefix + coralStateTopicName,
      toDashboardPrefix + algaeShootTimeTopicName,
      toDashboardPrefix + coralShootTimeTopicName,
      toDashboardPrefix + resetAlgaeFaceTopicName,
      toDashboardPrefix + resetCoralBranchTopicName,
      // autoPathTopicName,
      autoPathOptionsTopicName,
      // autoPathDefaultTopicName,
      autoPathActiveTopicName,
      // autoModeTopicName,
      autoModeOptionsTopicName,
      // autoModeDefaultTopicName,
      autoModeActiveTopicName,
      commodoreStateTopicName
    ],
    false,
    true,
    0.02
  );

  ntClient.publishTopic(toRobotPrefix + selectedCoralLevelTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + selectedCoralBranchTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + selectedAlgaeFaceTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + selectedPickupSideTopicName, "int");
  ntClient.publishTopic(toRobotPrefix + selectedDtpTopicName, "boolean");
  ntClient.publishTopic(toRobotPrefix + selectedDealgaefyDtpTopicName, "boolean");
  ntClient.publishTopic(toRobotPrefix + selectedAlgaeShootDtpTopicName, "boolean");
  ntClient.publishTopic(autoModeSelectedTopicName, "string");
  ntClient.publishTopic(autoPathSelectedTopicName, "string");
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
 * Function to update the DEalgaefy DTP value and radio buttons
 * @param {*} newValue 
 */
function updateDealgaefyDtpValue(newValue) {
  $("input[name='dealgaefydrivetopose'] + label").css("background", "");

  // Update the radio buttons and labels based on the new value
  if (selectedDealgaefyDtp === newValue) {
      $("label[for='dealgaefydrivetopose_" + String(newValue) + "']").css("background", "red");
  } else {
      $("input[name='dealgaefydrivetopose'][id='dealgaefydrivetopose_" + newValue + "']").prop('checked', true);
      $("label[for='dealgaefydrivetopose_" + String(newValue) + "']").css("background", "red");
      // currentDtpValue = newValue;
      selectedDealgaefyDtp = newValue;
  }
  // Send the updated value over NetworkTables
  sendBooleanToRobot(selectedDealgaefyDtpTopicName, selectedDealgaefyDtp);
}

/**
 * Function to update the Shoot Algae DTP value and radio buttons
 * @param {*} newValue
 */
function updateAlgaeShootDtpValue(newValue) {
  $("input[name='algaeshootdrivetopose'] + label").css("background", "");

  // Update the radio buttons and labels based on the new value
  if (selectedAlgaeShootDtp === newValue) {
      $("label[for='algaeshootdrivetopose_" + String(newValue) + "']").css("background", "red");
  } else {
      $("input[name='algaeshootdrivetopose'][id='algaeshootdrivetopose_" + newValue + "']").prop('checked', true);
      $("label[for='algaeshootdrivetopose_" + String(newValue) + "']").css("background", "red");
      // currentDtpValue = newValue;
      selectedAlgaeShootDtp = newValue;
  }
  // Send the updated value over NetworkTables
  sendBooleanToRobot(selectedAlgaeShootDtpTopicName, selectedAlgaeShootDtp);
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
function updateCoralBranchValue(newValue, fromRobot = false, reset = false) {
  if((selectedCoralBranch == newValue && !fromRobot) || reset) {
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
function updateAlgaeFaceValue(newValue, fromRobot = false, reset = false) {
  if((selectedAlgaeFace == newValue && !fromRobot) || reset) {
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
      target  = newValue;
      // target  = String.fromCharCode(64+(newValue*2)-1)+ String.fromCharCode(64+(newValue*2));
    
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


// Function to populate dropdowns dynamically
function populateDropdown(dropdownId, hiddenInputId, options) {
  let dropdownMenu = $(dropdownId);
  dropdownMenu.empty(); // Clear existing items
  options.forEach(option => {
      let newValue = option.toLowerCase().replace(/\s+/g, "-"); // Convert text to a safe value
      let newItem = `<li><a class="dropdown-item" href="#" data-value="${newValue}">${option}</a></li>`;
      dropdownMenu.append(newItem);
  });

  // Ensure all dropdown items update the button text & hidden input
  dropdownMenu.find(".dropdown-item").off("click").on("click", function () {
      let selectedText = $(this).text();
      let selectedValue = $(this).attr("data-value");

      let button = $(dropdownId).closest(".dropdown").find(".dropdown-toggle");

      // Update button text and store the selected value
      button.text(selectedText);
      $(hiddenInputId).val(selectedValue);

      console.log("Selected ("+dropdownId+")", selectedValue);
  });
}

// Function to update dropdown selection from the network
function updateDropdownSelection(dropdownId, hiddenInputId, option) {
  let button = $(dropdownId).closest(".dropdown").find(".dropdown-toggle");
  let newValue = option.toLowerCase().replace(/\s+/g, "-"); // Convert text to a safe value

  // Find the matching option in the dropdown
  let matchingOption = $(dropdownId).find(`.dropdown-item[data-value="${newValue}"]`);
  console.log("Matching Option:", matchingOption);
  if (matchingOption.length > 0) {
      let selectedText = matchingOption.text();

      // Update button text and hidden input
      button.text(selectedText);

      $(hiddenInputId).val(newValue);

      button.removeClass("btn-danger").addClass("btn-success");

      console.log("Network Updated:", newValue, "-- Dirty Flag OFF --");
  } else {
      console.warn("Value not found in dropdown:", newValue);
  }
}

function UpdateRobotState(newState) {
  let $eventList = $("#states-list");
  
  // Add new event to the end of the list
  $eventList.append(`<li class="list-group-item">${newState}</li>`);

  // Keep only the last 10 events
  if ($eventList.children().length > 10) {
      $eventList.children().first().remove();
  }

  // Scroll to the latest event
  let container = $("#states-log-container");
  container.scrollTop(container[0].scrollHeight);
}

function toggleAutoDropdown(isTeleop) {
  if (isTeleop) {
      $("#automode-wrapper, #trajectory-wrapper").show(); // Show dropdowns
  } else {
      $("#automode-wrapper, #trajectory-wrapper").hide(); // Hide dropdowns
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

/**
 * Send a boolean value to the robot
 * @param {*} topic 
 * @param {*} value 
 */
function sendStringToRobot(topic, value) {  
  if (typeof ntClient !== 'undefined' && ntClient.addSample) {
    ntClient.addSample(toRobotPrefix + topic, value);
    console.log("Sending to robot: ", toRobotPrefix + topic, value);
  } else {
    console.error("ntClient or ntClient.addSample is not defined");
  }
}

/**
 * Send a boolean value to the robot
 * @param {*} topic 
 * @param {*} value 
 */
function sendStringAbsoluteTopicToRobot(topic, value) {  
  if (typeof ntClient !== 'undefined' && ntClient.addSample) {
    ntClient.addSample(topic, value);
    console.log("Sending to robot: ", topic, value);
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

  $("input[name='dealgaefydrivetopose']").click(function (event) {
    console.log("DealgaefyDTP button clicked");
      updateDealgaefyDtpValue($(this).attr("value"));
  });

  $("input[name='algaeshootdrivetopose']").click(function (event) {
    console.log("AlgaeShoot DTP button clicked");
      updateAlgaeShootDtpValue($(this).attr("value"));
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

  // Handle the dropdowns
  $(document).on("click", ".dropdown-menu .dropdown-item", function () {
    let selectedText = $(this).text(); 
    let selectedValue = $(this).attr("data-value"); 
    let dropdownButton = $(this).closest(".dropdown").find(".dropdown-toggle"); 
    let hiddenInput = $(this).closest(".dropdown").next("input[type='hidden']"); 

    // Update the button text and store the value
    dropdownButton.text(selectedText);
    hiddenInput.val(selectedValue);
    console.log("Selected Value:", selectedText);

    // Change button color to warning (red)
    dropdownButton.removeClass("btn-success").addClass("btn-danger");

    // Get the dropdown ID to differentiate
    let dropdownId = dropdownButton.attr("id");

    // Call the mock server with different arguments based on dropdown
    if (dropdownId === "automode") {
        console.log("Sending Auto Mode Selection to Server:", selectedText, "-- Dirty Flag ON --");
        sendStringAbsoluteTopicToRobot(autoModeSelectedTopicName, selectedText);
    } else if (dropdownId === "trajectory") {
        console.log("Sending Trajectory Selection to Server:", selectedText, "-- Dirty Flag ON --");
        sendStringAbsoluteTopicToRobot(autoPathSelectedTopicName, selectedText);
    } else {
        console.log("Unknown dropdown selected:", selectedText, "-- Dirty Flag ON --");
    }

        
});

});