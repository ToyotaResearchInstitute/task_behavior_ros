// define a tag according to the Docker tag rules https://docs.docker.com/engine/reference/commandline/tag/
// the hash sign (#) is problematic when using it in bash, instead of working around this problem, just replace all
// punctuation with dash (-)
def projectShortName = "task_behavior_ros"
def githubOrg = "ToyotaResearchInstitute"
def dockerTag = "${env.BRANCH_NAME}-${env.BUILD_NUMBER}".toLowerCase().replaceAll("\\p{Punct}", "-").replaceAll("\\p{Space}", "-")

def buildLink = "<${env.BUILD_URL}|${env.JOB_NAME} ${env.BUILD_NUMBER}>"

node {
    timestamps {
        ansiColor('xterm') {
            try {
                properties properties: [
                        [$class: 'BuildDiscarderProperty', strategy: [$class: 'LogRotator', artifactDaysToKeepStr: '', artifactNumToKeepStr: '', daysToKeepStr: '30', numToKeepStr: '50']],
                        [$class: 'GithubProjectProperty', displayName: '', projectUrlStr: "https://github.com/$githubOrg/$projectShortName"],
                        pipelineTriggers([pollSCM('H/2 * * * *')])
                ]

                slackSend message: "build $buildLink started"
                stage('checkout') {
                    withEnv(["PATH+WSTOOL=${tool 'wstool'}/bin"]) {
                        sh """
                        rm -fr catkin_ws
                        """
                        dir('catkin_ws/src'){
                          sh """
                          wstool init .
                          """
                          dir("$projectShortName"){
                              checkout scm
                              sh """
                              wstool merge "$projectShortName".rosinstall
                              wstool up
                              """
                          }
                        }
                    }
                }
                stage('get_deps') {
                    withEnv(["PATH+ROSDEP=${tool 'rosdep'}/bin"]) {
                        sh """
                        rosdep update
                        rosdep install --from-paths catkin_ws/src --ignore-src --rosdistro=indigo -y
                        """
                    }
                }
                stage('build') {
                    withEnv(["PATH+CATKIN=${tool 'catkin'}/bin"]) {
                        sh """
                        . /opt/ros/indigo/setup.sh
                        catkin_make install -C catkin_ws
                        """
                    }
                }
                stage('test') {
                    withEnv(["PATH+CATKIN=${tool 'catkin'}/bin"]) {
                        sh """
                        . catkin_ws/install/setup.sh
                        catkin_make run_tests -C catkin_ws
                        catkin_test_results
                        """
                    }
                }
                slackSend color: 'good', message: "$buildLink completed successfully"
            } catch(Exception e) {
                slackSend color: 'danger', message: "build $buildLink failed"
                error "error building, ${e.getMessage()}"
            }
        }
    }
}
