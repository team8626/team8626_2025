$(document).ready(function () {
    // let previousChecked = $("input[name='intakeside']:checked").attr("id"); // Store initially checked button
    let currentValue = ""; // Store initially checked button

    // When any radio button in the intakeside group is clicked
    $("input[name='intakeside']").click(function (event) {
        console.log("Previous option: " + (currentValue ? currentValue : ""));

        $("input[name='intakeside'] + label").css("background", "");

        if(currentValue === $(this).attr("id")) {
            $(this).prop('checked', false);
            currentValue = "";
        } else {
            $(this).prop('checked', true);
            $("label[for='" + $(this).attr("id") + "']").css("background", "red");
            currentValue = $(this).attr("id");
        }
        console.log("Selected option: " + currentValue);
    });
});
