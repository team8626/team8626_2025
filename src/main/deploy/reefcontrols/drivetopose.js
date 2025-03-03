let currentDtpValue = ""; // Store initially checked button

// window.updateDtpValue = updateDtpValue;

// Function to update the DTP value and radio buttons
function updateDtpValue(newValue) {
    // Remove background color from all labels associated with the drivetopose group
    $("input[name='drivetopose'] + label").css("background", "");

    // Update the radio buttons and labels based on the new value
    if (currentDtpValue === newValue) {
        $("label[for='" + newValue + "']").css("background", "red");
    } else {
        $("input[name='drivetopose'][id='" + newValue + "']").prop('checked', true);
        $("label[for='" + newValue + "']").css("background", "red");
        currentDtpValue = newValue;
    }

    // Send the updated value over NetworkTables
    if (typeof window.ntClient !== 'undefined' && window.ntClient.addSample) {
        window.ntClient.addSample("/UIDashboard/ToRobot/" + window.dtpTopicName, !currentDtpValue);
    } else {
        console.error("ntClient or ntClient.addSample is not defined");
    }
    
    // console.log("Updated DTP to: " + currentDtpValue);
    return currentDtpValue;
}

$(document).ready(function () {
    // let previousChecked = $("input[name='intakeside']:checked").attr("id"); // Store initially checked button

    // When any radio button in the intakeside group is clicked
    $("input[name='drivetopose']").click(function (event) {
        updateDtpValue($(this).attr("id"));
    });
});
