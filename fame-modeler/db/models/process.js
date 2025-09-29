var mongoose = require('mongoose');
var Schema = mongoose.Schema;


var processSchema = new Schema({
    id: { type: String, required: true, unique: true },
    name: { type: String, required: true },
    process: { type: Schema.Types.Mixed, required: true },
},
    {
        versionKey: false
    });


var Process = mongoose.model('Process', processSchema);
module.exports = Process;