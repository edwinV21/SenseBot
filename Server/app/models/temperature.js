var mongoose     = require('mongoose');
var Schema       = mongoose.Schema;

var TempSchema   = new Schema({
    temperature: Number,
    danger: Boolean,
    time: { type: Date, default: Date.now }
});

module.exports = mongoose.model('Temperature', TempSchema);