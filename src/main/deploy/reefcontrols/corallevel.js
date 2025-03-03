$(document).ready(function () {
    // let previousChecked = $("input[name='intakeside']:checked").attr("id"); // Store initially checked button
    let currentCoralLevelValue = ""; // Store initially checked button

    // When any radio button in the intakeside group is clicked
    $("input[name='corallevel']").click(function (event) {
        console.log("Previous corallevel: " + (currentCoralLevelValue ? currentCoralLevelValue : ""));

        $("input[name='corallevel'] + label").css("background", "");

        if(currentCoralLevelValue === $(this).attr("id")) {
            $("label[for='" + $(this).attr("id") + "']").css("background", "red");
        } else {
            $(this).prop('checked', true);
            $("label[for='" + $(this).attr("id") + "']").css("background", "red");
            currentCoralLevelValue = $(this).attr("id");
        }
        console.log("Selected corallevel: " + currentCoralLevelValue);
    });
});
