#ifndef PROGRAMOPTIONS_H
#define PROGRAMOPTIONS_H
#include <QtCore>
#include <QRegularExpression>
class GenCmds{
public:
    GenCmds(const QStringList& cmds, const QStringList& vars): m_cmds(cmds), m_vars(vars)
    {
        int index = 0;
        for(const auto& str: cmds)
        {
            m_cmds[index].replace("\{", "{");
            m_cmds[index].replace("\}", "}");
            for(const auto& key: vars)
            {
                QRegularExpression regex(key);
                QRegularExpressionMatch match = regex.match(str);
                if (match.hasMatch())
                {
                    m_indexes.push_back(index);
                }
            }
            ++index;
        }
    }

    QStringList operator()(const QStringList& args)
    {
        if(args.size() != m_indexes.size())
        {
            qDebug() << "not valid args";
            return m_cmds;
        }
        for(int i=0; i<args.size(); ++i)
        {
            m_cmds[m_indexes[i]].replace(m_vars[i], args[i]);
        }
        return m_cmds;
    }

private:
    QVector<int> m_indexes;
    QStringList m_cmds, m_vars;
};



class ProgramOptions
{
public:
    ProgramOptions()
    {
        QString filePath = ":/config.json";
        qDebug() << "[+] parsing json file = " << parseJsonFile(filePath);

    }

    int parseJsonFile(const QString& filePath)
    {
        // Open the JSON file
          QFile jsonFile(filePath);
          if (!jsonFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
              qDebug() << "Failed to open JSON file:" << jsonFile.errorString();
              return 1;
          }

          // Read the contents of the JSON file
          QByteArray jsonData = jsonFile.readAll();
          jsonFile.close();

          // Parse the JSON data
          QJsonParseError parseError;
          QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData, &parseError);
          if (parseError.error != QJsonParseError::NoError) {
              qDebug() << "Failed to parse JSON file:" << parseError.errorString();
              return 1;
          }

          // Make sure the JSON data is an object
          if (!jsonDoc.isObject()) {
              qDebug() << "JSON data is not an object";
              return 1;
          }
          // create json object
          jsonObject = jsonDoc.object();
          return 0;
    }

    GenCmds getCLIcmd(const QString& key)
    {
        auto cand = getItem("system");
        auto cli = cand.value(key).toVariant().toMap();
        auto cmd = cli["cmd"].toStringList();
        auto vars = cli["vars"].toStringList();
        return GenCmds(cmd, vars);
    }


    QStringList getControllerNames()
    {
        auto cand = getItem("controller").toVariantMap().toStdMap();

        QStringList result;
        for (const auto name: cand)
        {
           result << name.first;
        }
        return result;
    }

    QStringList getControllerCmds()
    {
        QStringList results;
        auto cand = getItem("controller");
        for(auto item: getControllerNames())
        {
            auto obj = cand.value(item).toVariant().toMap();
            results << obj["cmd"].toString();
        }
        return results;
    }



private:
    QJsonObject jsonObject;
    QJsonObject getItem(const QString& field) const
    {
        return jsonObject.value(field).toObject();
    }
};

#endif // PROGRAMOPTIONS_H
