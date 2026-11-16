import QtQuick
import QtTest
import "../../.." as Gcs
TestCase {
    name:"FlightRecordEntry"
    visible:true;width:440;height:800
    when:windowShown
    Component {id:recordingPanel;Gcs.RecordingPanel {}}
    Component {id:signalSpy;SignalSpy {}}
    function test_read_button_emits_without_recording_command(){
        let actions=0
        const recorder={elapsedSeconds:0,autoEnabled:false,fresh:false,busy:false,autoText:"",autoReady:false,canStart:false,canStop:false,phaseText:"未录制",recording:false,bagMegabytes:0,outputDirectory:"",message:"",start:function(){actions++},stop:function(){actions++}}
        const panel=createTemporaryObject(recordingPanel,this,{recorder:recorder,width:390,collapsed:false})
        verify(panel!==null)
        const spy=createTemporaryObject(signalSpy,this,{target:panel,signalName:"readRecordsRequested"})
        const button=findChild(panel,"readOnboardFlightRecords")
        verify(button!==null);verify(button.enabled)
        wait(50);mouseClick(button,button.width/2,button.height/2)
        compare(spy.count,1);compare(actions,0)
    }
}
