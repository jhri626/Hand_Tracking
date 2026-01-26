/**
 * HMD_init.cpp
 * Initializes OpenXR session, OpenGL context, and VR tracking components for HMD and Vive trackers.
 * Handles system setup, reference spaces, swapchains, and action binding for VR hand tracking application.
 */


#define XR_EXTENSION_PROTOTYPES
#include "HMD.h"
#include <Windows.h>
#include <iostream>
#include <cstring>
// #include <spdlog/spdlog.h>
#include <thread>
#include "utils.h"
#include "HMD_number.h"





/**
 * Initializes the OpenXR system for head-mounted display.
 * Retrieves and validates the system ID required for subsequent OpenXR operations.
 */
bool HMD::initSystem() 
{
    // Prepare the system information structure for a head-mounted display.
    XrSystemGetInfo systemInfo{ XR_TYPE_SYSTEM_GET_INFO };
    systemInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;

    // Retrieve the system ID for the OpenXR instance.
    XrResult result = xrGetSystem(xrInstance, &systemInfo, &xrSystemId);
    if (XR_FAILED(result)) {
        std::cout << "[error] Failed to get OpenXR system!" << std::endl;
        return false;
    }

    // Check if the obtained system ID is valid.
    if (xrSystemId == XR_NULL_SYSTEM_ID) {
        std::cout << "[error] OpenXR system ID is invalid!" << std::endl;
        return false;
    }
    return true;
}

/**
 * Initializes OpenGL context with Windows-specific setup.
 * Creates rendering window, device context, and OpenGL rendering context for VR rendering.
 */
bool HMD::initOpenGL() {
    // If the window does not exist, create it.
    if (!hWnd && !CreateRenderWindow(hWnd)) {
        std::cerr << "No window" << std::endl;
        return false;
    }
    // Obtain the device context from the window.
    hDC = GetDC(hWnd);
    if (hDC == nullptr) {
        std::cerr << "Failed to get Device Context." << std::endl;
        return false;
    }

    // Define the pixel format descriptor.
    PIXELFORMATDESCRIPTOR pfd = {};
    pfd.nSize      = sizeof(PIXELFORMATDESCRIPTOR);
    pfd.nVersion   = 1;
    pfd.dwFlags    = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.iLayerType = PFD_MAIN_PLANE;

    // Choose the best pixel format that matches the descriptor.
    int pixelFormat = ChoosePixelFormat(hDC, &pfd);
    if (pixelFormat == 0) {
        std::cerr << "Failed to choose a pixel format." << std::endl;
        return false;
    }
    // Set the chosen pixel format for the device context.
    if (!SetPixelFormat(hDC, pixelFormat, &pfd)) {
        std::cerr << "Failed to set the pixel format."<<GetLastError() << std::endl;
        return false;
    }

    // Create the OpenGL rendering context.
    hGLRC = wglCreateContext(hDC);
    if (hGLRC == nullptr) {
        std::cerr << "Failed to create OpenGL Rendering Context." << std::endl;
        return false;
    }
    std::cerr << "OpenGL context created successfully. HGLRC: " << hGLRC << std::endl;

    // Make the created context current.
    if (!wglMakeCurrent(hDC, hGLRC)) {
        std::cerr << "Failed to make OpenGL context current." << std::endl;
        return false;
    }
    
    return true;
}


/**
 * Creates OpenXR instance and session with required extensions.
 * Sets up hand tracking, Vive tracker interaction, and OpenGL bindings for the VR application.
 */
bool HMD::CreateOpenXRInstanceAndSession() {
    // List of required extension names.
    const char* extensionNames[] = {
        XR_HTCX_VIVE_TRACKER_INTERACTION_EXTENSION_NAME,
        XR_EXT_HAND_TRACKING_EXTENSION_NAME,      // Hand tracking extension
        XR_KHR_OPENGL_ENABLE_EXTENSION_NAME        // OpenGL enable extension
        // XR_KHR_COMPOSITION_LAYER_COLOR_EXTENSION_NAME // for frame debug remove it later
    };

    if (!gladLoadGL()) {
        std::cerr << "[error] Failed to load GL functions\n";
        return false;
    }

    std::cerr << "Create session start"<< std::endl;

    // Setup the OpenXR instance creation info.
    XrInstanceCreateInfo xrInstanceCreateInfo = { XR_TYPE_INSTANCE_CREATE_INFO };
    std::strcpy(xrInstanceCreateInfo.applicationInfo.applicationName, "ViveFocusHandTracking");
    xrInstanceCreateInfo.applicationInfo.applicationVersion = 1;
    std::strcpy(xrInstanceCreateInfo.applicationInfo.engineName, "CustomEngine");
    xrInstanceCreateInfo.applicationInfo.engineVersion = 1;
    xrInstanceCreateInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;
    xrInstanceCreateInfo.enabledExtensionCount = sizeof(extensionNames) / sizeof(extensionNames[0]);
    xrInstanceCreateInfo.enabledExtensionNames = extensionNames;

    // Create the OpenXR instance.
    XrResult result = xrCreateInstance(&xrInstanceCreateInfo, &xrInstance);
    if (result != XR_SUCCESS) {
        std::cerr << "Failed to create OpenXR instance." << std::endl;
        return false;
    }
        std::cerr << "Create instance"<< std::endl;

    xrGetInstanceProcAddr(
        xrInstance,
        "xrEnumerateViveTrackerPathsHTCX",
        (PFN_xrVoidFunction*)&pfnEnumerateViveTrackerPathsHTCX
    );

    if (!pfnEnumerateViveTrackerPathsHTCX) {
        std::cerr << "[warn] Vive Tracker extension not supported by this runtime." << std::endl;
    }


    // Initialize the OpenXR system.
    if (!initSystem()) {
        std::cerr << "Failed to initialize OpenXR system!" << std::endl;
        return false;
    }

    std::cerr << "init system"<< std::endl;

    // Retrieve the function pointer for obtaining OpenGL graphics requirements.
    PFN_xrGetOpenGLGraphicsRequirementsKHR pfn_xrGetOpenGLGraphicsRequirementsKHR = nullptr;
    result = xrGetInstanceProcAddr(
        xrInstance, "xrGetOpenGLGraphicsRequirementsKHR",
        reinterpret_cast<PFN_xrVoidFunction*>(&pfn_xrGetOpenGLGraphicsRequirementsKHR)
    );
    if (result != XR_SUCCESS || pfn_xrGetOpenGLGraphicsRequirementsKHR == nullptr) {
        std::cerr << "Failed to get xrGetOpenGLGraphicsRequirementsKHR function pointer." << std::endl;
        std::cerr << "Error: " << GetLastError() << std::endl;
        std::cerr << "Result: " << result << std::endl;
        return false;
    }

    // Query the OpenGL graphics requirements.
    XrGraphicsRequirementsOpenGLKHR graphicsRequirements = { XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR };
    result = pfn_xrGetOpenGLGraphicsRequirementsKHR(xrInstance, xrSystemId, &graphicsRequirements);
    if (result != XR_SUCCESS) {
        std::cerr << "Failed to get OpenGL graphics requirements. Error: " << result << std::endl;
        return false;
    }

    // Ensure that the OpenGL context is correctly initialized.
    if (hDC == nullptr || hGLRC == nullptr) {
        std::cerr << "Error: OpenGL context is not initialized properly." << std::endl;
        return false;
    }

    // Setup session creation info and graphics binding.
    XrSessionCreateInfo xrSessionCreateInfo = { XR_TYPE_SESSION_CREATE_INFO };
    xrSessionCreateInfo.systemId = xrSystemId;
    
    XrGraphicsBindingOpenGLWin32KHR graphicsBinding{};
    graphicsBinding.type = XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR;
    graphicsBinding.hDC = hDC;
    graphicsBinding.hGLRC = hGLRC;
    xrSessionCreateInfo.next = &graphicsBinding;

    // Debug prints for context values.
    std::cerr << "hDC value: " << hDC << std::endl;
    std::cerr << "hGLRC value: " << hGLRC << std::endl;

    if (hDC == nullptr || hGLRC == nullptr) {
        std::cerr << "Error: OpenGL context is not initialized properly." << std::endl;
        return false;
    }

    std::cerr << "xrSessionCreateInfo.systemId: " << xrSessionCreateInfo.systemId << std::endl;
    std::cerr << "xrSessionCreateInfo.next: " << xrSessionCreateInfo.next << std::endl;

    // Create the OpenXR session.
    result = xrCreateSession(xrInstance, &xrSessionCreateInfo, &xrSession);
    if (result != XR_SUCCESS) {
        std::cerr << "Failed to create OpenXR session. Error: " << result << std::endl;
        return false;
    }

    std::cerr << "create session"<< std::endl;
    return true;
}


/**
 * Creates an OpenXR reference space of specified type.
 * Reference spaces define coordinate systems for tracking positions in VR (e.g., local, stage, view).
 */
bool HMD::CreateReferenceSpace(XrReferenceSpaceType type, XrSpace &outSpace){
    // Create a reference space for the session.
    XrReferenceSpaceCreateInfo referenceSpaceCreateInfo = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
    referenceSpaceCreateInfo.referenceSpaceType = type;
    referenceSpaceCreateInfo.poseInReferenceSpace.orientation.w = 1.0f;
    referenceSpaceCreateInfo.poseInReferenceSpace.position = { 0.0f, 0.0f, 0.0f };

    XrResult result = xrCreateReferenceSpace(xrSession, &referenceSpaceCreateInfo, &outSpace);
    if (result != XR_SUCCESS) {
        std::cerr << "Failed to create OpenXR space." << std::endl;
        return false;
    }
    return true;
}


/**
 * Begins the OpenXR session by attaching action sets and waiting for the session to be ready.
 * Polls events until the session reaches READY state, then creates reference spaces for tracking.
 */
bool HMD::beginOpenXRSession() {

    if (xrInstance == XR_NULL_HANDLE) {
        std::cerr << "[error] xrInstance is NULL! Cannot start session." << std::endl;
        return false;
    }

    XrSessionActionSetsAttachInfo attachInfo{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
    attachInfo.countActionSets = 1;
    attachInfo.actionSets = &trackerActionSet;

    if (XR_FAILED(xrAttachSessionActionSets(xrSession, &attachInfo))) {
        std::cerr << "[Error] Failed to attach ActionSets. Inputs will not work.\n";
        return false;
    }
    std::cout << "[Info] ActionSets attached successfully.\n";

    XrResult result;
    XrInstanceProperties instanceProperties{ XR_TYPE_INSTANCE_PROPERTIES };
    result = xrGetInstanceProperties(xrInstance, &instanceProperties);
    if (XR_FAILED(result)) {
        std::cerr << "[error] xrInstance is invalid just before calling xrPollEvent(). Error code: " << result << std::endl;
        return false;
    }

    bool running = true;
    while (running) {
        XrEventDataBuffer event{XR_TYPE_EVENT_DATA_BUFFER};
        while (xrPollEvent(xrInstance, &event) == XR_SUCCESS) {
            std::cout << "Session state changed: " << currentSessionState << std::endl;
            if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
                auto* ev = reinterpret_cast<XrEventDataSessionStateChanged*>(&event);
                currentSessionState = ev->state;
                std::cout << "Session state changed: " << currentSessionState << std::endl;
                if (currentSessionState == XR_SESSION_STATE_READY) {
                    XrSessionBeginInfo si{XR_TYPE_SESSION_BEGIN_INFO}; si.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                    XrResult res = xrBeginSession(xrSession, &si);
                    if (XR_FAILED(res)) {
                        std::cerr << "[error] Failed to begin OpenXR session! Error code: " << res << std::endl;
                        return false;
                    }
                    std::cout << "Session started. Creating reference space..." << std::endl;
                    CreateReferenceSpace(XR_REFERENCE_SPACE_TYPE_LOCAL, worldSpace);
                    CreateReferenceSpace(XR_REFERENCE_SPACE_TYPE_VIEW, hmdSpace);
                    std::cout << "Reference space created." << std::endl;
                    return true;
                }
                if (currentSessionState == XR_SESSION_STATE_STOPPING || currentSessionState == XR_SESSION_STATE_EXITING) running = false;
            }
            event = {XR_TYPE_EVENT_DATA_BUFFER};
        }
        
    }

    // session not ready
    std::cerr << "[error] OpenXR session is not ready!" << std::endl;
    return false;
}


/**
 * Creates an OpenXR swapchain for rendering VR frames.
 * Allocates image buffers with specified dimensions and format for stereoscopic rendering.
 */
bool HMD::CreateSwapchain(uint32_t width,
                          uint32_t height,
                          XrSwapchain& outSwapchain,
                          std::vector<XrSwapchainImageOpenGLKHR>& outImages)
{
    
    uint32_t formatCount = 0;
    xrEnumerateSwapchainFormats(xrSession, 0, &formatCount, nullptr);
    std::vector<int64_t> formats(formatCount);
    xrEnumerateSwapchainFormats(xrSession, formatCount, &formatCount, formats.data());

    
    int64_t chosenFormat = formats[0];
    for (auto f : formats) {
        if (f == GL_SRGB8_ALPHA8 || f == GL_RGBA8) {
            chosenFormat = f;
            break;
        }
    }

    
    XrSwapchainCreateInfo sci{ XR_TYPE_SWAPCHAIN_CREATE_INFO };
    sci.usageFlags  = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
    sci.format      = chosenFormat;
    sci.sampleCount = 1;
    sci.width       = width;
    sci.height      = height;
    sci.faceCount   = 1;
    sci.arraySize   = 1;    
    sci.mipCount    = 1;

    
    XrResult r = xrCreateSwapchain(xrSession, &sci, &outSwapchain);
    if (XR_FAILED(r)) {
        std::cerr << "[error] xrCreateSwapchain: " << r << "\n";
        return false;
    }

    
    uint32_t imageCount = 0;
    xrEnumerateSwapchainImages(outSwapchain, 0, &imageCount, nullptr);

    
    outImages.resize(imageCount, { XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR });
    xrEnumerateSwapchainImages(
        outSwapchain,
        imageCount,
        &imageCount,
        reinterpret_cast<XrSwapchainImageBaseHeader*>(outImages.data())
    );

    return true;
}



/**
 * Initializes all swapchains required for rendering (main and multiple small swapchains).
 * Sets up frame buffers for the primary display and additional rendering targets.
 */
bool HMD::InitAllSwapchains() {
    
    if (!CreateSwapchain(
            HMDVariable::SWAPCHAIN_WIDTH,
            HMDVariable::SWAPCHAIN_HEIGHT,
            xrSwapchain,
            swapchainImages))
    {
        return false;
    }

    mainWidth  = HMDVariable::SWAPCHAIN_WIDTH;
    mainHeight = HMDVariable::SWAPCHAIN_HEIGHT;

    
    for (int i = 0; i < kSmallCount; ++i) {
        if (!CreateSwapchain(
                HMDVariable::SWAPCHAIN_WIDTH,
                HMDVariable::SWAPCHAIN_HEIGHT,
                smallSwapchains[i],
                smallImages[i]))
        {
            return false;
        }
        smallWidth[i]  = HMDVariable::SWAPCHAIN_WIDTH;
        smallHeight[i] = HMDVariable::SWAPCHAIN_HEIGHT;
    }

    std::cerr << "[init] main=" << mainWidth << "x" << mainHeight << "\n";
    for (int i = 0; i < kSmallCount; ++i)
        std::cerr << "[init] small["<<i<<"]="<<smallWidth[i]<<"x"<<smallHeight[i]<<"\n";

    std::cout << "InitAllSwapchains: created main + " << kSmallCount << " small swapchains\n";
    return true;
}



/**
 * Initializes action set and pose actions for Vive trackers.
 * Creates subaction paths for each tracker role and sets up pose input actions for tracking.
 */
bool HMD::InitTrackerActions()
{
    // 1) Create Action Set
    XrActionSetCreateInfo setInfo{XR_TYPE_ACTION_SET_CREATE_INFO};
    strcpy(setInfo.actionSetName, "tracker_action_set");
    strcpy(setInfo.localizedActionSetName, "Tracker Action Set");
    setInfo.priority = 0;

    if (xrCreateActionSet(xrInstance, &setInfo, &trackerActionSet) != XR_SUCCESS) {
        std::cerr << "[error] Failed to create tracker ActionSet\n";
        return false;
    }


    trackerCount = (int)trackerRoleStrings.size();
    if (trackerCount > MAX_TRACKERS) {
        std::cerr << "[Error] Too many trackers defined in role list!\n";
        return false;
    }

    for (int i = 0; i < trackerCount; ++i) {
        
        if (XR_FAILED(xrStringToPath(xrInstance, trackerRoleStrings[i].c_str(), &trackerPaths[i]))) {
            std::cerr << "[Error] Failed to convert path string: " << trackerRoleStrings[i] << "\n";
            return false;
        }
    }

    // 2) Create Pose Action
    XrActionCreateInfo actInfo{XR_TYPE_ACTION_CREATE_INFO};
    actInfo.actionType = XR_ACTION_TYPE_POSE_INPUT;
    strcpy(actInfo.actionName, "tracker_pose");
    strcpy(actInfo.localizedActionName, "Tracker Pose");

    actInfo.countSubactionPaths = trackerCount;
    actInfo.subactionPaths = trackerPaths;

    if (xrCreateAction(trackerActionSet, &actInfo, &trackerPoseAction) != XR_SUCCESS) {
        std::cerr << "[error] Failed to create tracker Pose Action\n";
        return false;
    }

    return true;
}



/**
 * Binds tracker pose actions to HTC Vive tracker interaction profile.
 * Maps each tracker role to its corresponding grip pose input path for position tracking.
 */
bool HMD::BindTrackerAction()
{
    std::cout << "======= BindTrackerAction (Single Fixed) =======\n";

    
    XrPath profilePath;
    xrStringToPath(xrInstance, "/interaction_profiles/htc/vive_tracker_htcx", &profilePath);

    

    std::vector<XrActionSuggestedBinding> bindings;

    for (int i = 0; i < trackerCount; ++i) {
        
        std::string roleStr = trackerRoleStrings[i];
        
        
        std::string fullPathStr = roleStr + "/input/grip/pose";

        XrPath inputPath;
        if (XR_FAILED(xrStringToPath(xrInstance, fullPathStr.c_str(), &inputPath))) {
             std::cerr << "[Error] Failed to make path for: " << fullPathStr << "\n";
             return false;
        }
        
        
        XrActionSuggestedBinding binding{};
        binding.action = trackerPoseAction;
        binding.binding = inputPath;
        bindings.push_back(binding);

        std::cout << "[Bind] " << fullPathStr << "\n";
    }

    XrInteractionProfileSuggestedBinding suggested{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
    suggested.interactionProfile = profilePath;
    suggested.countSuggestedBindings = (uint32_t)bindings.size();
    suggested.suggestedBindings = bindings.data();

    
    XrResult res = xrSuggestInteractionProfileBindings(xrInstance, &suggested);
    if (XR_FAILED(res)) {
        std::cerr << "[Error] Failed to suggest bindings: " << res << "\n";
        return false;
    }

    std::cout << "[Success] Manually bound Left Foot path.\n";
    return true;
}



/**
 * Creates action spaces for each initialized tracker.
 * Action spaces allow querying tracker positions and orientations relative to reference spaces.
 */

bool HMD::CreateTrackerSpaces()
{
    for (int i = 0; i < trackerCount; ++i) {

        XrActionSpaceCreateInfo spaceInfo{
            XR_TYPE_ACTION_SPACE_CREATE_INFO
        };

        spaceInfo.action = trackerPoseAction;
        spaceInfo.subactionPath = trackerPaths[i];
        spaceInfo.poseInActionSpace.orientation = {0,0,0,1};
        spaceInfo.poseInActionSpace.position    = {0,0,0};

        if (xrCreateActionSpace(xrSession, &spaceInfo, &trackerSpaces[i]) != XR_SUCCESS) {
            std::cerr << "[error] Failed to create ActionSpace for tracker["<<i<<"]\n";
            return false;
        }
        else {
            std::cerr << "[Info] Create ActionSpace for tracker["<<i<<"]\n";
        }
    }

    return true;
}
