package com.iot.airqualityprojectweb;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.webkit.GeolocationPermissions;
import android.webkit.WebChromeClient;
import android.webkit.WebView;
import android.webkit.WebViewClient;

public class MainActivity extends AppCompatActivity {

    WebView navegador;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        navegador = (WebView) findViewById(R.id.webApp);

        /*crea un directori a l'arrel al mateix nivell que res anomenat assets,
        i dins col·loca les pàgines que es vulguin colocar dins d'aplicació*/

        navegador.getSettings().setJavaScriptEnabled(true);

        //per geolocalització de HTML5
        navegador.getSettings().setAppCacheEnabled(true);
        navegador.getSettings().setDatabaseEnabled(true);
        navegador.getSettings().setDomStorageEnabled(true);
        navegador.setWebChromeClient(new WebChromeClient() {
            public void onGeolocationPermissionsShowPrompt(String origin, GeolocationPermissions.Callback callback) {
                callback.invoke(origin, true, false);
            }
        });
        //per defecte ens obrirà Chrome, cal canviar-ho
        navegador.setWebViewClient(new WebViewClient());
        navegador.loadUrl("http://iot.airqualityproject.com:2222/ui/#!/0?socketid=S-_xLOGmZ0E46q9KAAAP");
        //navegador.loadData("<html><body>hola, mon!</body></html>", "text/html", "UTF-8");
//        navegador.loadData("<html><body><input type=\"button\" value=\"Hola\" onClick=\"mostrarToast('Hola Android!')\" />\n" +
//                "<script type=\"text/javascript\">\n" +
//                "function mostrarToast(toast) {\n" +
//                "Android.mostrarToast(toast);\n" +
//                "}\n" +
//                "</script></body></html>", "text/html", "UTF-8");
       //navegador.addJavascriptInterface (new WebAppInterface (this), "Android");
        //navegador.loadUrl("www.google.com");
    }
}