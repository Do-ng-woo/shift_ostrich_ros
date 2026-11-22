"""Add collision filters and observation sensors to a Gazebo SDF model."""
import subprocess
import tempfile
import xml.etree.ElementTree as ET

EXEMPT_LINKS = ()  # No robot-link collision exemptions (2026-11-08).


def prepare_observed_model(description):
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf') as source:
        source.write(description)
        source.flush()
        converted = subprocess.run(['gz', 'sdf', '-p', source.name],
                                   capture_output=True, text=True, check=True)
    sdf = ET.fromstring(converted.stdout)
    model = sdf.find('model')
    for link in model.findall('link'):
        name = link.get('name')
        ET.SubElement(link, 'self_collide').text = 'true'
        mask = 1
        for collision in link.findall('collision'):
            surface = collision.find('surface')
            if surface is None:
                surface = ET.SubElement(collision, 'surface')
            contact = surface.find('contact')
            if contact is None:
                contact = ET.SubElement(surface, 'contact')
            for tag in ('collide_bitmask', 'category_bitmask'):
                element = contact.find(tag)
                if element is None:
                    element = ET.SubElement(contact, tag)
                element.text = str(mask)
            sensor = ET.SubElement(link, 'sensor', name=name+'_contacts', type='contact')
            ET.SubElement(sensor, 'always_on').text = 'true'
            ET.SubElement(sensor, 'update_rate').text = '100'
            ET.SubElement(sensor, 'topic').text = '/model/shift_ostrich/contacts'
            sensor_contact = ET.SubElement(sensor, 'contact')
            ET.SubElement(sensor_contact, 'collision').text = collision.get('name')
            ET.SubElement(sensor_contact, 'topic').text = '/model/shift_ostrich/contacts'
    plugin = ET.SubElement(model, 'plugin', filename='gz-sim-pose-publisher-system',
                           name='gz::sim::systems::PosePublisher')
    for tag, value in {'publish_link_pose': 'true', 'publish_model_pose': 'true',
                       'publish_nested_model_pose': 'true', 'use_pose_vector_msg': 'true',
                       'update_frequency': '100'}.items():
        ET.SubElement(plugin, tag).text = value
    return ET.tostring(sdf, encoding='unicode')
