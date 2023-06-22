DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ROSPKG_ENVIRONMENT_NAME=${DIR##*/}
echo $ROSPKG_ENVIRONMENT_NAME

echo "moving folders"
[ -f agri_environment_template ] && mv agri_environment_template $ROSPKG_ENVIRONMENT_NAME
[ -f resources/agri_environment_template ] && mv resources/agri_environment_template resources/$ROSPKG_ENVIRONMENT_NAME

echo "updating setup files"
sed -i 's/agri_environment_template/$ROSPKG_ENVIRONMENT_NAME/g' setup.cfg
sed -i 's/agri_environment_template/$ROSPKG_ENVIRONMENT_NAME/g' setup.py
sed -i 's/agri_environment_template/$ROSPKG_ENVIRONMENT_NAME/g' CMakeLists.txt
sed -i 's/agri_environment_template/$ROSPKG_ENVIRONMENT_NAME/g' package.xml

