const mongoose = require('mongoose');
const message = require('./models/ros_msg');
const processModel = require('./models/process'); 
const fs = require("fs-extra");
const { parse } = require("csv-parse");

const url = "mongodb://localhost:27017/fame";

// Connect to MongoDB
mongoose.connect(url);

const conn = mongoose.connection;
conn.on('error', console.error.bind(console, 'Connection error: '));
conn.once('open', function () {
  console.log('Connection OK!');
});

// Read and save CSV data
async function readCSVAndSave(filePath, Model, mapper) {
  const records = [];

  return new Promise((resolve, reject) => {
    fs.createReadStream(filePath)
      .pipe(parse({ delimiter: ",", from_line: 2, relax_quotes: true }))
      .on("data", function (row) {
        records.push(mapper(row));
      })
      .on("end", async function () {
        try {
          await Model.insertMany(records);
          console.log(`Finished processing ${filePath}`);
          resolve();
        } catch (err) {
          reject(err);
        }
      })
      .on("error", function (error) {
        reject(error);
      });
  });
}

// Map rows for ros_msg.csv
const mapRosMsgRow = (row) => ({
  type: row[0],
  class: row[1],
  payload: row[2]
});

// Map rows for process.csv
const mapProcessRow = (row) => ({
  id: row[0],
  name: row[1],
  process: row[2]
});

// Process both CSV files concurrently
async function processCSVFiles() {
  try {
    await Promise.all([
      readCSVAndSave("./data/ros_msg.csv", message, mapRosMsgRow),
      readCSVAndSave("./data/process.csv", processModel, mapProcessRow)
    ]);
    console.log("Both CSV files successfully processed.");
  } catch (error) {
    console.error("Error processing files:", error);
  } finally {
    mongoose.connection.close();
  }
}

processCSVFiles();
