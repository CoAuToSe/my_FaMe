import { XMLParser, XMLBuilder } from "fast-xml-parser";

const backURL = 'http://localhost:9000';


async function send(processData) {
    try {
        const response = await fetch(backURL + '/processes', {
            method: "POST",
            headers: { 'Content-Type': 'application/json'},
            body: JSON.stringify(processData),
        });
        const result = await response.json();
        console.log("Success:", result);
        alert('Call activity successfully saved!');
        close();
    } catch (error) {
        console.error("Error:", error);
    }
}

export function storeCallAct(modeler) {
    var outProcess = ''
    var newProcess = {}
    // Export bpmn as string
    async function getXML() {
        try {
            const xml = await modeler.saveXML({ format: false });
            return xml;
        } catch (err) {
            console.log(err);
        }
    }

    getXML().then(value => {
        outProcess = value.xml

        // Get process name
        const options = {
            ignoreAttributes: false,
            attributeNamePrefix: "@_"
        };
        const parser = new XMLParser(options);
        let name = parser.parse(outProcess)['bpmn:definitions']['bpmn:process']['@_id'].replace('Process_', '');
        newProcess = {
            id: name,
            name: name,
            process: '"' +  outProcess + '"'
        }

        send(newProcess);
    });
}

