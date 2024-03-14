#include <gst/gst.h>
#include <iostream>
// #include <gst/rtp/gstrtpbuffer.h>
#include <thread>
#include <string>
#include <fstream>
#include <vector>
#include <unordered_map>

#define DEFAULT_RTSP_PORT "8554"

typedef struct myDataTag {
    GstElement *pipeline;
    GstElement *rtspsrc;
    GstElement *depayloader;
    GstElement *decoder;
    GstElement *jifmux;
    GstElement *sink;
} myData_t;

typedef struct CustomData_ {
    GstElement *pipeline;
    GstElement *rtspsrc;
    GstElement *depayloader;
    GstElement *conv;
    GstElement *sink;
} CustomData;



/* pad added handler */
static void pad_added_handler(GstElement *src, GstPad *new_pad, myData_t *pThis)
{
    GstPad *sink_pad = gst_element_get_static_pad(pThis->depayloader, "sink");
    GstPadLinkReturn ret;
    GstCaps *new_pad_caps = NULL;
    GstStructure *new_pad_struct = NULL;
    const gchar *new_pad_type = NULL;

    g_print("Received new pad '%s' from '%s':\n", GST_PAD_NAME(new_pad), GST_ELEMENT_NAME(src));

    /* Check the new pad's name */
    if (!g_str_has_prefix(GST_PAD_NAME(new_pad), "recv_rtp_src_")) {
        g_print("  It is not the right pad.  Need recv_rtp_src_. Ignoring.\n");
        goto exit;
    }

    /* If our converter is already linked, we have nothing to do here */
    if (gst_pad_is_linked(sink_pad)) {
        g_print(" Sink pad from %s already linked. Ignoring.\n", GST_ELEMENT_NAME(src));
        goto exit;
    }

    /* Check the new pad's type */
    new_pad_caps = gst_pad_get_current_caps(new_pad);
    new_pad_struct = gst_caps_get_structure(new_pad_caps, 0);
    new_pad_type = gst_structure_get_name(new_pad_struct);

    /* Attempt the link */
    ret = gst_pad_link(new_pad, sink_pad);
    if (GST_PAD_LINK_FAILED(ret)) {
        g_print("  Type is '%s' but link failed.\n", new_pad_type);
        GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pThis->pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");
    
    } else {
        GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pThis->pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");
        g_print("  Link succeeded (type '%s').\n", new_pad_type);
    }

exit:
      /* Unreference the new pad's caps, if we got them */
    if (new_pad_caps != NULL)
        gst_caps_unref(new_pad_caps);

    /* Unreference the sink pad */
    gst_object_unref(sink_pad);
}


/* pad added handler */
static void pad_added_handler_tof(GstElement *src, GstPad *new_pad, CustomData *pThis)
{
    GstPad *sink_pad = gst_element_get_static_pad(pThis->depayloader, "sink");
    GstPadLinkReturn ret;
    GstCaps *new_pad_caps = NULL;
    GstStructure *new_pad_struct = NULL;
    const gchar *new_pad_type = NULL;

    g_print("Received new pad '%s' from '%s':\n", GST_PAD_NAME(new_pad), GST_ELEMENT_NAME(src));

    /* Check the new pad's name */
    if (!g_str_has_prefix(GST_PAD_NAME(new_pad), "recv_rtp_src_")) {
        g_print("  It is not the right pad.  Need recv_rtp_src_. Ignoring.\n");
        goto exit;
    }

    /* If our converter is already linked, we have nothing to do here */
    if (gst_pad_is_linked(sink_pad)) {
        g_print(" Sink pad from %s already linked. Ignoring.\n", GST_ELEMENT_NAME(src));
        goto exit;
    }

    /* Check the new pad's type */
    new_pad_caps = gst_pad_get_current_caps(new_pad);
    new_pad_struct = gst_caps_get_structure(new_pad_caps, 0);
    new_pad_type = gst_structure_get_name(new_pad_struct);

    /* Attempt the link */
    ret = gst_pad_link(new_pad, sink_pad);
    if (GST_PAD_LINK_FAILED(ret)) {
        g_print("  Type is '%s' but link failed.\n", new_pad_type);
        GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pThis->pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");
    
    } else {
        GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pThis->pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");
        g_print("  Link succeeded (type '%s').\n", new_pad_type);
    }

exit:
      /* Unreference the new pad's caps, if we got them */
    if (new_pad_caps != NULL)
        gst_caps_unref(new_pad_caps);

    /* Unreference the sink pad */
    gst_object_unref(sink_pad);
}

static int cnt = 0;
static GstPadProbeReturn cb_jpg_img (GstPad * pad, GstPadProbeInfo * info, gpointer user_data) {
    // printf("ALED: %d\n", cnt);
    cnt = 0;

    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER (info);

  	int buf_size = gst_buffer_get_size (buffer);
	printf("------ jpeg buf_size = %d\n", buf_size);

	return GST_PAD_PROBE_OK;

}

static GstPadProbeReturn cb_raw_img (GstPad * pad, GstPadProbeInfo * info, gpointer user_data) {

    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER (info);

  	int buf_size = gst_buffer_get_size (buffer);
	printf("------ raw buf_size = %d\n", buf_size);


	return GST_PAD_PROBE_OK;
}


typedef struct {
    uint64_t timestamp;
    float    x;
    float    y;
    float    z;
    float    q[4]; /* Quaternion */
} RtspMeta;

typedef struct {
    uint64_t timestamp;
    float    q[4]; /* Quaternion */
} RtspMetaTOF;

static GstPadProbeReturn new_rtp_buff (GstPad * pad, GstPadProbeInfo * info, gpointer user_data) {
    printf("aled\n");
	char data[1400];
    RtspMeta m;
    myData_t appData = *(myData_t*)user_data;
	guint wordlen;
	guint16 bits;
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER (info);
  	int buf_size = gst_buffer_get_size (buffer);
	gst_buffer_extract(buffer, 0, data, 52);
    // printf("%x\n", (unsigned char)data[1]);
    // std::cout << "ALED " << (unsigned)data[1] << std::endl;
    if((unsigned char)data[1] == 0x9A) {
        gst_buffer_extract(buffer, buf_size-sizeof(RtspMeta), &m, sizeof(RtspMeta));

        char meta_char[150]; // Taille suffisamment grande pour contenir les données JSON
        memset(meta_char, 0, sizeof(meta_char)); // Initialisation du tampon à zéro

        unsigned int stamp_sec = 0;
        stamp_sec = m.timestamp >> 32;
        unsigned int stamp_nsec = 0;
        stamp_nsec = m.timestamp & 0xffffffff;

        // Formatage des données en format JSON
        sprintf(meta_char,
            "{ sec: %u, nsec: %u, x: %.2f, y: %.2f, z: %.2f, q: [%.2f, %.2f, %.2f, %.2f] }",
            stamp_sec, stamp_nsec, m.x, m.y, m.z, m.q[0], m.q[1], m.q[2], m.q[3]);
        std::cout << meta_char << std::endl;
        gst_tag_setter_add_tags(
            GST_TAG_SETTER(appData.jifmux), GST_TAG_MERGE_REPLACE, GST_TAG_DESCRIPTION,
            meta_char, NULL
        );


    } else {

        cnt++;    
    }
	return GST_PAD_PROBE_OK;
}


int pipe_th(int id)
{
    GMainLoop *loop;
    myData_t appData;
    
    std::string _uri = std::string("rtsp://192.168.1.15:8554/pgm" + std::to_string(id));
    std::string _location = std::string("pgm" + std::to_string(id) + "/frame%06d.jpg");


    loop = g_main_loop_new(NULL, FALSE);

    appData.pipeline = gst_pipeline_new( std::string("videoclient"+std::to_string(id)).c_str());
    appData.rtspsrc = gst_element_factory_make("rtspsrc", std::string("rtspsrc"+std::to_string(id)).c_str());
 
    g_object_set(G_OBJECT(appData.rtspsrc), "location", _uri.c_str(), NULL);
    g_object_set(G_OBJECT(appData.rtspsrc), "latency", 200, NULL);

    appData.depayloader = gst_element_factory_make("rtpjpegdepay", std::string("depayloader"+std::to_string(id)).c_str());
    appData.jifmux = gst_element_factory_make("jifmux", std::string("jifmux"+std::to_string(id)).c_str());

    appData.sink = gst_element_factory_make("multifilesink", std::string("sink"+std::to_string(id)).c_str());
    g_object_set(G_OBJECT(appData.sink), "location", _location.c_str(), NULL);

    GstPad* depay_padsink = gst_element_get_static_pad(appData.depayloader, "sink");

    gst_pad_add_probe (depay_padsink, GST_PAD_PROBE_TYPE_BUFFER,
      (GstPadProbeCallback) &new_rtp_buff, &appData, nullptr);
      

    if (!appData.pipeline || !appData.rtspsrc || !appData.depayloader || !appData.jifmux || !appData.sink) {
        g_printerr ("Not all elements could be created.\n");
        return -1;
    }

    // then add all elements together
    gst_bin_add_many(GST_BIN(appData.pipeline), appData.rtspsrc, appData.depayloader, appData.jifmux, appData.sink, NULL);

#if 0 // link done one by one 
    if (!gst_element_link(appData.depayloader, appData.jifmux)) {
        g_printerr ("Elements 1 error.\n");
        gst_object_unref (appData.pipeline);
        return -1;
    }

    if (!gst_element_link(appData.jifmux, appData.sink)) {
        g_printerr ("Elements 2 could not be linked.\n");
        gst_object_unref (appData.pipeline);
        return -1;
    }
#else
    // link everythink after source
    if (!gst_element_link_many(appData.depayloader, appData.jifmux, appData.sink, NULL)) {
        g_printerr ("Elements could not be linked.\n");
        gst_object_unref (appData.pipeline);
        return -1;
    }
#endif 

    /*
     * Connect to the pad-added signal for the rtpbin.  This allows us to link
     * the dynamic RTP source pad to the depayloader when it is created.
     */
    g_signal_connect(appData.rtspsrc, "pad-added", G_CALLBACK(pad_added_handler), &appData);

    /* Set the pipeline to "playing" state*/
    int ret = gst_element_set_state(appData.pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(appData.pipeline);
        return -1;
    }

    /* Iterate */
    g_print("Running...\n");
    g_main_loop_run(loop);

    /* Out of the main loop, clean up nicely */
    g_print("Returned, stopping playback\n");
    gst_element_set_state(appData.pipeline, GST_STATE_NULL);

    g_print("Deleting pipeline\n");
    gst_object_unref(GST_OBJECT(appData.pipeline));

    return 0;
}


std::vector<int> LUT(255, -1);
 
void init_LUT(){
    std::string filename = "LUT.txt";
 
    std::ifstream file(filename);
 
    if (!file.is_open()) {
        std::cerr << "Erreur lors de l'ouverture du fichier." << std::endl;
        return;
    }
 
    int index = 0;
    int _value;
 
    // Utiliser une unordered_map pour suivre les indices de la première occurrence de chaque valeur
    std::unordered_map<int, int> firstIndexMap;
 
    // Lire les valeurs depuis le fichier
    while (file >> _value) {
        // Si la valeur n'a pas été rencontrée auparavant, enregistrez son index
        if (firstIndexMap.find(_value) == firstIndexMap.end()) {
            firstIndexMap[_value] = index;
        }
        index++;
    }
 
    file.close();
 
    // Remplir le vecteur des indices des premières occurrences
    for (const auto& entry : firstIndexMap) {
        if (entry.first >= 0 && entry.first < 255) {
            LUT[entry.first] = entry.second;
        }
    }
 
    // Utiliser le vecteur comme nécessaire
    for (int i = 0; i < LUT.size(); ++i) {
        // std::cout << "La première occurrence de la valeur " << i << " se trouve à l'index : " << LUT[i] << std::endl;
    }
}
int frame_number = 0;
 

GstPadProbeReturn depayloaderSrcCallBack(
    GstPad* pad, GstPadProbeInfo* info, gpointer user_data
){
    GstMapInfo map;
    auto* buf = gst_pad_probe_info_get_buffer(info);
    int buf_size = gst_buffer_get_size(buf);
    printf("Size of buffer %d\n", buf_size);

   if (gst_buffer_map(buf, &map, GST_MAP_READ)) {
        guint8* pixels = reinterpret_cast<guint8*>(map.data);

        std::string filename = "tof/frame_" + std::to_string(frame_number++) + ".txt"; 
        FILE* file = fopen(filename.c_str(), "w");
        int iter = 0;
        int last = 20;

        for (int y = 0; y < 172; ++y) {
            for (int x = 0; x < 224*3; x += 3) {

                // if((y * 224*3 + x) > 115583 - last) {
                //     printf("%d converted to %d\n", pixels[y * 224 + x], LUT.at(pixels[y * 224 + x]));
                // }
                guint16 value = LUT.at(pixels[y * 224*3 + x]); 
                fprintf(file, "%d ", value);
                
            }
            fprintf(file, "\n"); 

        }
        uint8_t meta[16];
        int it = 0;

        printf("End with last %ld: ",buf_size - (buf_size - (2*sizeof(float)+2*sizeof(unsigned))*3));
        for(int i = buf_size - (2*sizeof(float)+2*sizeof(unsigned int))*3; i < buf_size; i+=3) {
            printf("%d, ", pixels[i]);
            meta[it] = pixels[i];
            it++;
        }   
        
        printf("\n");
        
        float qy = 0;
        float qw = 0;
        
        unsigned int stamp_sec = 0;
        unsigned int stamp_nsec = 0;

        memcpy(&stamp_sec, meta, sizeof(unsigned int));
        memcpy(&stamp_nsec, meta+sizeof(unsigned int), sizeof(unsigned int));

        memcpy(&qy, meta+8, sizeof(float));
        memcpy(&qw, meta+8+sizeof(float), sizeof(float));

        printf("sec = %u\nnsec = %u\n", stamp_sec, stamp_nsec);
        printf("qy = %f, qw = %f\n", qy, qw);
        fprintf(file, "%u %u %f %f\n", stamp_sec, stamp_nsec, qy,qw);

        // printf("NB iter %d\n", iter);
        fclose(file);
        gst_buffer_unmap(buf, &map);
    } else {
        g_printerr("Unable to map buffer\n");
    }

    // getTOFMetadata(pad, info, user_data);

    return GST_PAD_PROBE_OK;
}



int pipe_tof_th() {

    CustomData appData;
    GMainLoop *loop;
    GstPad *pad;

    std::string _uri = std::string("rtsp://192.168.1.15:8554/tof");
    std::string _location = std::string("tmp_tof/frame%d.ppm");

    loop = g_main_loop_new(NULL, FALSE);

    appData.pipeline = gst_pipeline_new("flexx2_receiver");
    appData.rtspsrc = gst_element_factory_make("rtspsrc", "rtspsrc");
 
    g_object_set(G_OBJECT(appData.rtspsrc), "location", _uri.c_str(), NULL);
    g_object_set(G_OBJECT(appData.rtspsrc), "latency", 200, NULL);

    appData.depayloader = gst_element_factory_make("rtpvrawdepay", "depay");
    
    appData.conv = gst_element_factory_make("videoconvert", "conv");

    appData.sink = gst_element_factory_make("autovideosink", "sink");
    // appData.sink = gst_element_factory_make("multifilesink", "sink");
// 
    // g_object_set(G_OBJECT(appData.sink), "location", _location.c_str(), NULL);
      

    if (!appData.pipeline || !appData.rtspsrc || !appData.depayloader || !appData.conv || !appData.sink) {
        g_printerr ("Not all elements could be created.\n");
        return -1;
    }

    // then add all elements together
    gst_bin_add_many(GST_BIN(appData.pipeline), appData.rtspsrc, appData.depayloader, appData.conv, appData.sink, NULL);

    pad = gst_element_get_static_pad(appData.depayloader, "src");
    gst_pad_add_probe(
        pad, GST_PAD_PROBE_TYPE_BUFFER,
        (GstPadProbeCallback)&depayloaderSrcCallBack, NULL, nullptr
    );

    gst_object_unref(pad);


    if (!gst_element_link_many(appData.depayloader, appData.conv, appData.sink, NULL)) {
        g_printerr ("Elements could not be linked.\n");
        gst_object_unref (appData.pipeline);
        return -1;
    }

    /*
     * Connect to the pad-added signal for the rtpbin.  This allows us to link
     * the dynamic RTP source pad to the depayloader when it is created.
     */
    g_signal_connect(appData.rtspsrc, "pad-added", G_CALLBACK(pad_added_handler_tof), &appData);
    // g_signal_connect(appData.rtspsrc, "pad-added", G_CALLBACK(pad_added_handler), appData.depayloader);
    // g_signal_connect (appData.rtspsrc, "pad-added", G_CALLBACK (pad_added_handler), &appData);

    // GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");

    /* Set the pipeline to "playing" state*/
    int ret = gst_element_set_state(appData.pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(appData.pipeline);
        return -1;
    }

    /* Iterate */
    g_print("Running...\n");
    g_main_loop_run(loop);

    /* Out of the main loop, clean up nicely */
    g_print("Returned, stopping playback\n");
    gst_element_set_state(appData.pipeline, GST_STATE_NULL);

    g_print("Deleting pipeline\n");
    gst_object_unref(GST_OBJECT(appData.pipeline));

    return 0;
}

int main(int argc, char *argv[]){

    gst_init(&argc, &argv);

    init_LUT();

    std::thread pgm1_th (pipe_th, 1);     
    std::thread pgm2_th (pipe_th, 2);     
    std::thread tof_th  (pipe_tof_th);  

    pgm1_th.join();
    pgm2_th.join();
    tof_th.join();


    return 0;
}
