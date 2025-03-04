$(document).ready(function () {
    const hexContainer = $(".hexagon-container"); // Select hexagon container

    // Labels for the buttons (A to L)
    const corallabels = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"];
    const algaelabels = ["AB", "CD", "EF", "GH", "IJ", "KL"];

    for (let i = 0; i < 12; i++) {
        // Create label wrapper
        let label = $("<label>")
            .addClass("hexagon-coral")
            .css("--angle", `${(i * -30)+183}deg`);

        // Create radio button
        let radio = $("<input>").attr({
            class: "btn-check",
            type: "radio",
            name: "coralGroup",
            id: `branch_${corallabels[i]}`, // Unique ID
            value: `${i+1}`
        });

        // Create the visual dot with letter inside
        // let dot = $("<span>").addClass("dot").text(labels[i]); // Add label inside button
        // let dot = $("<label>").addClass("btn btn-secondary").for(`branch_${labels[i]}`).text(labels[i]); // Add label inside button
        // let coral = $("<span>").addClass("btn btn-secondary rounded-circle coral").text(corallabels[i]); // Add label inside button
        let coral = $("<span>").addClass("btn btn-secondary rounded-circle coral").append($("<span>").addClass("coral-wrapper").text(corallabels[i]));

        // Append elements
        label.append(radio).append(coral);
        hexContainer.append(label);
    }


    for (let i = 0; i < 6; i++) {
        // Create label wrapper
        let label = $("<label>")
            .addClass("hexagon-algae")
            .css("--angle", `${(i * -60)+153}deg`);

        // Create radio button
        let radio = $("<input>").attr({
            class: "btn-check",
            type: "radio",
            name: "algaeGroup",
            id: `branch_${algaelabels[i]}`, // Unique ID
            value: `${i+1}`
        });

        // Create the visual dot with letter inside
        // let dot = $("<span>").addClass("dot").text(labels[i]); // Add label inside button
        // let dot = $("<label>").addClass("btn btn-secondary").for(`branch_${labels[i]}`).text(labels[i]); // Add label inside button
        let algae = $("<span>").addClass("btn btn-secondary rounded-circle algae").append($("<span>").addClass("algae-wrapper").text(algaelabels[i]));

        // Append elements
        label.append(radio).append(algae);
        hexContainer.append(label);
    }

});
