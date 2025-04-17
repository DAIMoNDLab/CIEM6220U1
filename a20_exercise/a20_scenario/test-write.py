from lxml import etree

tree=etree.parse('test-xml-write.xml')
root=tree.getroot()

CACC_default_values_dict={'minGap':0.25,'accel':1.5,'decel':2,'tau':1,'speedControlGainCACC':-0.4,'gapClosingControlGainGap':0.005,'gapClosingControlGainGapDot':0.05, 
                          'gapControlGainGap':0.45,'gapControlGainGapDot':0.0125,'collisionAvoidanceGainGap':0.45,'collisionAvoidanceGainGapDot':0.05}

CACC_user_values_dict=CACC_default_values_dict
CACC_user_values_dict['speedControlGainCACC']=-0.6

HV_default_values_dict={'minGap':0.25,'accel':1.5,'decel':2,'sigma':0.5,'tau':1}

HV_user_values_dict=HV_default_values_dict
HV_user_values_dict['sigma']=0.8

for element in root.iter("vType"):
    attribs=element.attrib
    if element.get("id")=="BA_AV":
        #Adapt parameters based on user input
        for k in CACC_default_values_dict.keys():
            if k in CACC_user_values_dict:
                attribs[k]=str(CACC_user_values_dict[k])
            else:
                attribs[k]=str(CACC_default_values_dict[k])
        

    if element.get("id")=="BA_cars":
        #Adapt parameters based on user input
        for k in HV_default_values_dict.keys():
            if k in HV_user_values_dict:
                attribs[k]=str(HV_user_values_dict[k])
            else:
                attribs[k]=str(HV_default_values_dict[k])

tree.write('test-xml-write.xml')