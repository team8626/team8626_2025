// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

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
      console.log("Alliance color: " + allianceColor);
    } else if (topic.name === toDashboardPrefix + isAutoTopicName) {
      isAuto = value;
      updateMatchTime(matchTime, isAuto);
      console.log("Is Auto: " + isAuto);
    } else if (topic.name === toDashboardPrefix + matchTimeTopicName) {
      matchTime = value;
      updateMatchTime(matchTime, isAuto);
    } else if (topic.name === toDashboardPrefix + allowedCoralLevelsTopicName) {
      allowedCoralLevels = value;
      console.log("Allowed coral levels: " + allowedCoralLevels);
    } else if (topic.name === toDashboardPrefix + selectedCoralLevelTopicName) {
      selectedCoralLevel = value;
      updateCoralLevelValue(selectedCoralLevel);
      console.log("Selected coral level: " + selectedCoralLevel);
    } else if (topic.name === toDashboardPrefix + selectedCoralBranchTopicName) {
      selectedCoralBranch = value;
      console.log("Selected coral branch: " + selectedCoralBranch);
    } else if (topic.name === toDashboardPrefix + selectedAlgaeFaceTopicName) {
      selectedAlgaeFace = value;
      console.log("Selected algae face: " + selectedAlgaeFace);
    } else if (topic.name === toDashboardPrefix + selectedPickupSideTopicName) {
      selectedPickupSide = value;
      updateIntakeSideValue(selectedPickupSide);
      console.log("Selected pickup side: " + selectedPickupSide);
    } else if (topic.name === toDashboardPrefix + selectedDtpTopicName) {
      selectedDtp = value;
      updateDtpValue(selectedDtp);
      console.log("Input DTP: " + selectedDtp);
    } else if (topic.name === toDashboardPrefix + algaeStateTopicName) {
      algaeStatus = value;
      updateAlgaeState(algaeStatus);
      console.log("Algae state: " + algaeStatus);
    } else if (topic.name === toDashboardPrefix + coralStateTopicName) { 
      coralStatus = value;
      updateCoralState(coralStatus);
      console.log("Coral state: " + coralStatus);
    } else if (topic.name === toDashboardPrefix + algaeShootTimeTopicName) {
      algaeShootTime = value;
      updateAlgaeShootTime(algaeShootTime);
      console.log("Algae shoot time: " + algaeShootTime);
    } else if (topic.name === toDashboardPrefix + coralShootTimeTopicName) {
      coralShootTime = value;
      updateCoralShootTime(coralShootTime);
      console.log("Coral shoot time: " + coralShootTime);
    } else {
      return;
    }
    updateUI();
  },
  () => {
    $("body").css("background-color", "#111529");
  },
  () => {
    // Disconnected
    $("body").css("background-color", "red");  }
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
      toDashboardPrefix + coralShootTimeTopicName
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

// Bind DTP Selector
// bind(document.getElementsByClassName("drivetopose")[0], () => {
//   selectedDtp = updateDtpValue();
//   console.log("Selected DTP: " + !selectedDtp);
//   if(selectedDtp == true) {
//     ntClient.addSample(toRobotPrefix + dtpTopicName, true);
//     console.log("Output DTP: TRUE");

//   } else {
//     ntClient.addSample(toRobotPrefix + dtpTopicName, false);
//     console.log("Output DTP: FALSE");
//   }
// });

  // // Update counter highlight
  // Array.from(document.getElementsByClassName("counter-area")).forEach(
  //   (element, index) => {
  //     if (index > 0 && selectedLevel === index - 1) {
  //       element.classList.add("active");
  //     } else {
  //       element.classList.remove("active");
  //     }
  //   }
  // );

  // // Update level counts
  // let rpLevelCount = 0;
  // Array.from(document.getElementsByClassName("counter")).forEach(
  //   (element, index) => {
  //     if (index === 0) {
  //       element.innerText = l1State;
  //       if (l1State >= 5) rpLevelCount++;
  //     } else {
  //       let count = 0;
  //       let levelState = [l2State, l3State, l4State][index - 1];
  //       for (let i = 0; i < 12; i++) {
  //         if (((1 << i) & levelState) > 0) {
  //           count++;
  //         }
  //       }
  //       element.innerText = count === 12 ? "\u2713" : count;
  //       if (count >= 5) rpLevelCount++;
  //     }
  //   }
  // );

  // // Update coral buttons
  // Array.from(document.getElementsByClassName("branch")).forEach(
  //   (element, index) => {
  //     let levelState = [l2State, l3State, l4State][selectedLevel];
  //     if (((1 << index) & levelState) > 0) {
  //       element.classList.add("active");
  //     } else {
  //       element.classList.remove("active");
  //     }
  //   }
  // );

  // // Update algae buttons
  // Array.from(document.getElementsByClassName("algae")).forEach(
  //   (element, index) => {
  //     if (((1 << index) & algaeState) > 0) {
  //       element.classList.add("active");
  //     } else {
  //       element.classList.remove("active");
  //     }
  //   }
  // );

  // Update coop button
  // let coopDiv = document.getElementsByClassName("coop")[0];
  // if (coopState) {
  //   coopDiv.classList.add("active");
  // } else {
  //   coopDiv.classList.remove("active");
  // }

  // Update RP flag
  // document.getElementsByClassName("flag")[0].hidden =
  //   isElims || rpLevelCount < (coopState ? 3 : 4);

  // Update elims state
  // if (isElims) {
  //   document.body.classList.add("elims");
  // } else {
  //   document.body.classList.remove("elims");
  // }


// ***** BUTTON BINDINGS *****

// function bind(element, callback) {
//   let lastActivation = 0;
//   let activate = () => {
//     if (new Date().getTime() - lastActivation > 500) {
//       callback();
//       lastActivation = new Date().getTime();
//     }
//   };

//   element.addEventListener("touchstart", activate);
//   element.addEventListener("click", activate);
//   element.addEventListener("contextmenu", (event) => {
//     event.preventDefault();
//     activate();
//   });
// }

// window.addEventListener("load", () => {
//   // Buttons to change selected level
//   Array.from(document.getElementsByClassName("counter-area")).forEach(
//     (element, index) => {
//       if (index > 0) {
//         bind(element, () => {
//           ntClient.addSample(toRobotPrefix + selectedLevelTopicName, index - 1);
//         });
//       }
//     }
//   );

//   // Coral toggle buttons
//   Array.from(document.getElementsByClassName("branch")).forEach(
//     (element, index) => {
//       bind(element, () => {
//         switch (selectedLevel) {
//           case 0:
//             ntClient.addSample(
//               toRobotPrefix + l2TopicName,
//               l2State ^ (1 << index)
//             );
//             break;
//           case 1:
//             ntClient.addSample(
//               toRobotPrefix + l3TopicName,
//               l3State ^ (1 << index)
//             );
//             break;
//           case 2:
//             ntClient.addSample(
//               toRobotPrefix + l4TopicName,
//               l4State ^ (1 << index)
//             );
//             break;
//         }
//       });
//     }
//   );

//   // Algae toggle buttons
//   Array.from(document.getElementsByClassName("algae")).forEach(
//     (element, index) => {
//       bind(element, () => {
//         ntClient.addSample(
//           toRobotPrefix + algaeTopicName,
//           algaeState ^ (1 << index)
//         );
//       });
//     }
//   );

  // // L1 count controls
  // bind(document.getElementsByClassName("subtract")[0], () => {
  //   if (l1State > 0) {
  //     ntClient.addSample(toRobotPrefix + l1TopicName, l1State - 1);
  //   }
  // });
  // bind(document.getElementsByClassName("add")[0], () => {
  //   ntClient.addSample(toRobotPrefix + l1TopicName, l1State + 1);
  // });

  
  // DTP Selector
  // bind(document.getElementsByClassName("drivetopose")[0], () => {
  //   console.log("Output DTP: " + !window.selectedDtp);
  //   ntClient.addSample(toRobotPrefix + dtpTopicName, !window.selectedDtp);
  // });
// });

// ***** REEF CANVAS *****

// window.addEventListener("load", () => {
//   const canvas = document.getElementsByTagName("canvas")[0];
//   const context = canvas.getContext("2d");

//   let render = () => {
//     const devicePixelRatio = window.devicePixelRatio;
//     const width = canvas.clientWidth;
//     const height = canvas.clientHeight;
//     canvas.width = width * devicePixelRatio;
//     canvas.height = height * devicePixelRatio;
//     context.scale(devicePixelRatio, devicePixelRatio);
//     context.clearRect(0, 0, width, height);

//     const corners = [
//       [width * 0.74, height * 0.9],
//       [width * 0.26, height * 0.9],
//       [width * 0.03, height * 0.5],
//       [width * 0.26, height * 0.1],
//       [width * 0.74, height * 0.1],
//       [width * 0.97, height * 0.5],
//     ];

//     context.beginPath();
//     corners.forEach((corner, index) => {
//       context.moveTo(width * 0.5, height * 0.5);
//       context.lineTo(...corner);
//     });
//     corners.forEach((corner, index) => {
//       if (index == 0) {
//         context.moveTo(...corner);
//       } else {
//         context.lineTo(...corner);
//       }
//     });
//     context.closePath();

//     context.strokeStyle = "black";
//     context.stroke();
//   };

//   render();
//   window.addEventListener("resize", render);
// });

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

function updateCoralBranchValue(newValue) {
  if(selectedCoralBranch === newValue) {
    $("input[name='coralGroup']").prop("checked", false); // Uncheck all radios
    $(this).prop('checked', false);
    selectedCoralBranch = 0;
  } else {
    $(this).prop('checked', true);
    selectedCoralBranch = newValue;
  }
  // Send the updated value over NetworkTables
  sendIntToRobot(selectedCoralBranchTopicName, selectedCoralBranch);
};

function updateAlgaeFaceValue(newValue) {
  if(selectedAlgaeFace === newValue) {
    $("input[name='algaeGroup']").prop("checked", false); // Uncheck all radios
    $(this).prop('checked', false);
    selectedAlgaeFace = 0;
  } else {
    $(this).prop('checked', true);
    selectedAlgaeFace = newValue;
  }
  // Send the updated value over NetworkTables
  sendIntToRobot(selectedAlgaeFaceTopicName, selectedAlgaeFace);
};

function updateCoralLevelValue(newValue) {
  // $("input[name='corallevel'] + label").css("background", "");

  if(selectedCoralLevel === newValue) {
    // $("label[for='corallevel_" + newValue + "']").css("background", "red");
  } else {
    $(this).prop('checked', true);
    // $("label[for='corallevel_" + newValue + "']").css("background", "red");
    selectedCoralLevel = newValue;
  }
  // Send the updated value over NetworkTables
  console.log("Sending coral level: " + selectedCoralLevel);
  sendIntToRobot(selectedCoralLevelTopicName, selectedCoralLevel);
};

// Function to update match time
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