// Pure DOM-state regression; never connects to a server or a robot.
const fs = require('fs'), vm = require('vm'), assert = require('assert');
const source = fs.readFileSync(require('path').join(__dirname, '../control_center/static/app.js'), 'utf8');
const render = source.slice(source.indexOf('function renderManipulation()'), source.indexOf("$('manip-run').onclick"));
function view(ready) {
  const elements = {}, textValues = {};
  const m = {available:true, active:false, ages:{readiness:0, adapter:0, base:0, wrist:0},
    readiness:{ready, errors:ready?[]:['READ_ONLY_MODE']}, base:{stopped:true,locked:false,reason:'UNLOCKED'},
    execution:{blockers:ready?[]:['실행기 준비 미완료'], arm_blockers:ready?[]:['실행기 준비 미완료'],
      handoff:{ready:true,task_id:'new-dock',message:'도킹 완료·실측 정지 인계 완료',
        coarse_position:{frame:'odom',xyz:[.3,.1,.6]}}},
    status:{task_id:'previous-job',stage:'COMPLETE',outcome:'SUCCESS'}};
  const context = {data:{connected:true,values:{manipulation:m}},servicePending:new Set(),
    text:(k,v)=>textValues[k]=v, $:k=>elements[k]||(elements[k]={})};
  vm.runInNewContext(render+';renderManipulation();',context);
  assert.equal(elements['manip-run'].disabled,!ready);
  assert.equal(textValues['manip-stage'],'도킹 완료·실측 정지 인계 완료');
  assert.equal(textValues['manip-task'],'작업 ID · new-dock');
  assert(textValues['manip-position'].includes('인계 좌표'));
  assert(textValues['manip-state'].includes(ready?'실행 가능':'팔 실행 준비 필요'));
}
view(false); view(true);
console.log('PASS: handed-off task shown independently; hardware-ready enables run, read-only keeps it disabled; previous result does not hide the new task');
