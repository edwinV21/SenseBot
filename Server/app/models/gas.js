var mongoose     = require('mongoose');
var Schema       = mongoose.Schema;

var GasSchema   = new Schema({
    gasLevel: Number,
    danger: Boolean,
    time: { type: Date, default: Date.now }
});

module.exports = mongoose.model('Gas', GasSchema);